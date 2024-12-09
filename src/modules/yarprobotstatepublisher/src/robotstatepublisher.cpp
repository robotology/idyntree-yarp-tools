/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file coordinator.cpp
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include <Eigen/Dense>
#include <yarp/math/Math.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/EigenHelpers.h>

#include "robotstatepublisher.h"

using namespace idyntree_yarp_tools;


bool YARPRobotStatePublisherModule::configureTransformServer(const std::string &name, const yarp::os::Searchable &rf)
{

    std::string namePrefix = rf.check("name-prefix", yarp::os::Value("")).asString();
    if (namePrefix.front() != '/')
    {
        namePrefix = '/' + namePrefix;
    }

    yarp::os::Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", rf.check("tf-device", yarp::os::Value("frameTransformClient")).asString());
    pTransformclient_cfg.put("filexml_option",  rf.check("tf-file", yarp::os::Value("ftc_yarp_only.xml")).asString());
    pTransformclient_cfg.put("ft_client_prefix", namePrefix + "/" + name + "/tf");
    if (rf.check("tf-remote"))
    {
        pTransformclient_cfg.put("ft_server_prefix", rf.find("tf-remote").asString());
    }
    pTransformclient_cfg.put("local_rpc", namePrefix + "/" + name + "/tf/local_rpc");

    m_tfPrefix = rf.check("tf-prefix", yarp::os::Value("")).asString();

    bool ok_client = m_ddtransformclient.open(pTransformclient_cfg);
    if (!ok_client)
    {
        yError()<<"Problem in opening the transformClient device";
        yError()<<"Is the transformServer YARP device running?";
        return false;
    }
    if (!m_ddtransformclient.view(m_iframetrans))
    {
        yError()<<"IFrameTransform I/F is not implemented";
        return false;
    }

    return true;
}

YARPRobotStatePublisherModule::YARPRobotStatePublisherModule(): m_iframetrans(nullptr),
                                                                m_usingNetworkClock(false),
                                                                m_baseFrameName(""),
                                                                m_baseFrameIndex(iDynTree::FRAME_INVALID_INDEX),
                                                                m_buf4x4(4,4)
{
}

bool YARPRobotStatePublisherModule::configure(yarp::os::ResourceFinder &rf)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string name;
    auto basicInfo = std::make_shared<BasicInfo>();

    {
        std::lock_guard<std::mutex> lock(basicInfo->mutex);
        basicInfo->name = rf.check("name", yarp::os::Value("yarprobotstatepublisher")).asString();
        name = basicInfo->name;
        basicInfo->robotPrefix = rf.check("robot", yarp::os::Value("icub")).asString();
    }

    std::string modelFileName=rf.check("model", yarp::os::Value("model.urdf")).asString();
    // Open the model
    std::string pathToModel=rf.findFileByName(modelFileName);

    if (pathToModel == "")
    {
        yError() << "Failed to find" << modelFileName;
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    bool ok = modelLoader.loadModelFromFile(pathToModel);

    ConnectionType connection = BasicConnector::RequestedType(rf, ConnectionType::STATE_EXT);

    switch (connection)
    {
    case ConnectionType::STATE_EXT:
    {
        std::shared_ptr<StateExtConnector> stateExtConnector = std::make_shared<StateExtConnector>();

        if (!stateExtConnector->configure(rf, basicInfo))
        {
            yError() << "Failed to configure the module to connect to the robot via the StateExt port.";
            return false;
        }

        m_connector = stateExtConnector;
        break;
    }

    case ConnectionType::REMAPPER:
    {
        std::shared_ptr<RemapperConnector> remapperConnector = std::make_shared<RemapperConnector>();

        if (!remapperConnector->configure(rf, modelLoader.model(), basicInfo))
        {
            yError() << "Failed to configure the module to connect to the robot via RemoteControlBoardRemapper.";
            return false;
        }

        m_connector = remapperConnector;
        break;
    }

    default:
        yError() << "The specified connector is not available for this module.";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(basicInfo->mutex);
        std::stringstream jointListInfo;
        jointListInfo << "Using the following joints:" << std::endl;
        for (const std::string& joint : basicInfo->jointList)
        {
            jointListInfo << "    - " << joint <<std::endl;
        }

        yInfo() << jointListInfo.str();

        if (!modelLoader.loadReducedModelFromFullModel(modelLoader.model(), basicInfo->jointList)) //The connectors take care of setting the joint list
        {
            yError() << "Failed to get reduced model.";
            return false;
        }

        // Resize the joint pos buffer
        m_jointPos.resize(basicInfo->jointList.size());

        // Initilize the joint pos buffer to Zero
        m_jointPos.zero();
        m_jointOffsets = m_jointPos;

        yarp::os::Value jointOffsetsValue = rf.find("joint-offsets-deg");
        if (jointOffsetsValue.isList())
        {
            yarp::os::Bottle* jointOffsetsList = jointOffsetsValue.asList();
            for (size_t i = 0 ; i < jointOffsetsList->size(); ++i)
            {
                yarp::os::Value offsetValue = jointOffsetsList->get(i);
                if (!offsetValue.isList())
                {
                    yWarning() << "The element at position" << i <<"in joint-offsets is not a list.";
                    continue;
                }
                yarp::os::Bottle* offsetBottle = offsetValue.asList();
                if (offsetBottle->size() != 2)
                {
                    yWarning() << "The element at position" << i <<"in joint-offsets-deg is not a list of two elements.";
                    continue;
                }

                if (!offsetBottle->get(0).isString())
                {
                    yWarning() << "The first element at position" << i <<"in joint-offsets-deg is not a string.";
                    continue;
                }

                if (!offsetBottle->get(1).isFloat64() && !offsetBottle->get(1).isInt64() && !offsetBottle->get(1).isInt32())
                {
                    yWarning() << "The second element at position" << i << "("<< offsetBottle->get(0).asString() << ") in joint-offsets-deg is not a number.";
                    continue;
                }

                auto jointIt = std::find(basicInfo->jointList.begin(), basicInfo->jointList.end(), offsetBottle->get(0).asString());

                if (jointIt == basicInfo->jointList.end())
                {
                    yWarning() << "Specified an offset for the joint" << offsetBottle->get(0).asString() << "but it is not in the joint list.";
                    continue;
                }

                yInfo() << "Adding an offset of" << offsetBottle->get(1).asFloat64() << "to" << offsetBottle->get(0).asString();

                m_jointOffsets(std::distance(basicInfo->jointList.begin(), jointIt)) = iDynTree::deg2rad(offsetBottle->get(1).asFloat64());
            }
        }
        else
        {
            yWarning() << "joint-offsets found, but it is not a list.";
        }
    }

    m_period=rf.check("period", yarp::os::Value(0.010)).asFloat64();
    m_treeType=rf.check("tree-type", yarp::os::Value("SHALLOW")).asString();
    if(m_treeType != "SHALLOW" && m_treeType != "DEEP")
    {
        yError("Wrong tree format. The only allowed values are \"SHALLOW\" or \"DEEP\"");
        return false;
    }

    ok = ok && m_kinDynComp.loadRobotModel(modelLoader.model());
    if (!ok || !m_kinDynComp.isValid())
    {
        yError()<<"Impossible to load file " << pathToModel;
        return false;
    }

    // Get the base frame information
    if (rf.check("base-frame"))
    {
        m_baseFrameName = rf.find("base-frame").asString();
    }
    else
    {
        // If base-frame is not passed, use the default base-frame of the model
        const iDynTree::Model& model = m_kinDynComp.model();
        m_baseFrameName = model.getLinkName(model.getDefaultBaseLink());
    }

    const iDynTree::Model& model = m_kinDynComp.model();
    m_baseFrameIndex = model.getFrameIndex(m_baseFrameName);

    if (m_baseFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError()<<"Impossible to find frame " << m_baseFrameName << " in the model";
        return false;
    }

    // Set reduced model option
    // By default TFs of all the frames in the model are streamed
    // If the option is present, only the TFs of the links are streamed to transform server
    this->reducedModelOption=rf.check("reduced-model");

    std::stringstream frameNameInfo;
    frameNameInfo << "Publishing the following frames:" << std::endl;
    // Set the size of the tf frames to be published
    size_t sizeOfTFFrames;
    if (this->reducedModelOption)
    {
        sizeOfTFFrames = model.getNrOfLinks();
    }
    else
    {
        sizeOfTFFrames = model.getNrOfFrames();
    }
    for (size_t frameIdx = 0; frameIdx < sizeOfTFFrames; frameIdx++)
    {
        frameNameInfo << "    - " << model.getFrameName(frameIdx) <<std::endl;
    }
    yInfo() << frameNameInfo.str();

    // If YARP is using a network clock, writing on a ROS topic is not working
    // Workaround: explicitly instantiate a network clock to read the time from gazebo
    if( yarp::os::NetworkBase::exists("/clock") )
    {
        m_usingNetworkClock = true;
        m_netClock.open("/clock");
    }

    if (!configureTransformServer(name, rf))
    {
        return false;
    }

    if (!m_connector->connectToRobot())
    {
        yError() << "Failed to connect to the robot";
        return false;
    }

    return true;
}


/************************************************************/
bool YARPRobotStatePublisherModule::close()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    // Close the connector
    if (m_connector)
    {
        m_connector->close();
    }

    if (m_ddtransformclient.isValid())
    {
        yInfo()<<"Closing the tf device";
        m_ddtransformclient.close();
        m_iframetrans = nullptr;
    }

    m_baseFrameIndex = iDynTree::FRAME_INVALID_INDEX;

    return true;
}


/************************************************************/
double YARPRobotStatePublisherModule::getPeriod()
{
    return m_period;
}



/************************************************************/
bool YARPRobotStatePublisherModule::updateModule()
{
    if (!m_useCallback)
    {
        onReadCallback();
    }

    if (!m_publishedOnce)
    {
        yWarningThrottle(5.0) << "No data published yet";
    }
    else
    {
        yInfoThrottle(10.0) << "YARPRobotStatePublisherModule running happily";
    }

    return true;
}

/************************************************************/
void YARPRobotStatePublisherModule::onReadCallback()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    // If configure was successful, parse the data
    if (m_baseFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        return;
    }

    if (!m_connector->getJointValues(m_jointPos))
        return;

    iDynTree::toEigen(m_jointPos) = iDynTree::toEigen(m_jointPos) + iDynTree::toEigen(m_jointOffsets);

    const iDynTree::Model& model = m_kinDynComp.model();

    // Set the updated joint positions
    m_kinDynComp.setJointPos(m_jointPos);

    // Set the size of the tf frames to be published
    size_t sizeOfTFFrames;
    if (this->reducedModelOption)
    {
        sizeOfTFFrames = model.getNrOfLinks();
    }
    else
    {
        sizeOfTFFrames = model.getNrOfFrames();
    }

    if (m_treeType == "SHALLOW")
    {
        // In shallow mode, we publish the position of each frame of the robot w.r.t. to the base frame of the robot
        for (size_t frameIdx=0; frameIdx < sizeOfTFFrames; frameIdx++)
        {
            if(m_baseFrameIndex == frameIdx)    // skip self-tranform
                continue;

            iDynTree::Transform base_H_frame = m_kinDynComp.getRelativeTransform(m_baseFrameIndex, frameIdx);
            iDynTree::toYarp(base_H_frame.asHomogeneousTransform(), m_buf4x4);
            m_iframetrans->setTransform(m_tfPrefix + model.getFrameName(frameIdx),
                                        m_tfPrefix + model.getFrameName(m_baseFrameIndex),
                                        m_buf4x4);
        }
    }
    else
    {
        // mode == DEEP
        // In deep mode, we need to distinguish the following cases:
        // For the frames that are frames of the link, we publish their location w.r.t. to their parent link
        // For the additional frames, we publish their location w.r.t. to the frame of the link to which they are
        // attached (note that this transform are actually constant)

        // The traversal is the data structure that contains information on which link is parent of which other link,
        // as in iDynTree the model is an undirected data structure
        iDynTree::Traversal traversal;

        // We generate a traversal using the base frame index
        m_kinDynComp.model().computeFullTreeTraversal(traversal, m_baseFrameIndex);

        bool setOk = false;

        //Processing joints instead of links since it's easier this way to distinguish between static transform and non static ones
        for (size_t jointIndex=0; jointIndex < model.getNrOfJoints(); jointIndex++)
        {
            auto currJoint = model.getJoint(jointIndex);
            iDynTree::LinkIndex parentLinkIndex = traversal.getParentLinkIndexFromJointIndex(model,jointIndex);//currJoint->getFirstAttachedLink();
            iDynTree::LinkIndex linkIndex = traversal.getChildLinkIndexFromJointIndex(model,jointIndex);//currJoint->getSecondAttachedLink();
            iDynTree::Transform parentLink_H_link = m_kinDynComp.getRelativeTransform(parentLinkIndex, linkIndex);
            iDynTree::toYarp(parentLink_H_link.asHomogeneousTransform(), m_buf4x4);

            if(currJoint->getNrOfDOFs() == 0) //Static transform
            {
                //To avoid setting a static transform more than once
                if(m_iframetrans->canTransform(m_tfPrefix + model.getFrameName(linkIndex),m_tfPrefix + model.getFrameName(parentLinkIndex)))
                {
                    continue;
                }
                setOk = m_iframetrans->setTransformStatic(m_tfPrefix + model.getFrameName(linkIndex),
                                                          m_tfPrefix + model.getFrameName(parentLinkIndex),
                                                          m_buf4x4);
            }
            else
            {
                setOk = m_iframetrans->setTransform(m_tfPrefix + model.getFrameName(linkIndex),
                                                    m_tfPrefix + model.getFrameName(parentLinkIndex),
                                                    m_buf4x4);
            }

            if(!setOk)
            {
                yInfo("The transformation between %s and %s cannot be set as %s",(m_tfPrefix + model.getFrameName(parentLinkIndex)).c_str(),
                      (m_tfPrefix + model.getFrameName(linkIndex)).c_str(),currJoint->getNrOfDOFs()==0?"static":"timed");
            }
        }

        // Process frames, only if the reduced model option is not passed
        if (!this->reducedModelOption)
        {
            // Process additional frames (that have all indexes between model.getNrOfLinks()+1 and model.getNrOfFrames()
            for (size_t frameIndex=model.getNrOfLinks(); frameIndex < model.getNrOfFrames(); frameIndex++)
            {
                iDynTree::LinkIndex linkIndex = m_kinDynComp.model().getFrameLink(frameIndex);
                iDynTree::Transform link_H_frame = m_kinDynComp.model().getFrameTransform(frameIndex);
                iDynTree::toYarp(link_H_frame.asHomogeneousTransform(), m_buf4x4);

                //To avoid setting a static transform more than once
                if(m_iframetrans->canTransform(m_tfPrefix + model.getFrameName(frameIndex),m_tfPrefix + model.getFrameName(linkIndex)))
                {
                    continue;
                }
                setOk = m_iframetrans->setTransformStatic(m_tfPrefix + model.getFrameName(frameIndex),
                                                          m_tfPrefix + model.getFrameName(linkIndex),
                                                          m_buf4x4);
                if(!setOk)
                    yInfo("The transformation between %s and %s cannot be set",(m_tfPrefix + model.getFrameName(linkIndex)).c_str(),
                          (m_tfPrefix + model.getFrameName(frameIndex)).c_str());
            }
        }
    }

    m_publishedOnce = true;

    return;
}

