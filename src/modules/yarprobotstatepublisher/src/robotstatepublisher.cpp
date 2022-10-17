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

#include <yarp/math/Math.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/yarp/YARPConversions.h>

#include "robotstatepublisher.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace idyntree_yarp_tools;

/************************************************************/
JointStateConnector::JointStateSubscriber::JointStateSubscriber()
{

}

JointStateConnector::JointStateSubscriber::~JointStateSubscriber()
{
    std::lock_guard<std::mutex> lock(m_mutex);
}

void JointStateConnector::JointStateSubscriber::attach(JointStateConnector *connector)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_connector = connector;
}

void JointStateConnector::JointStateSubscriber::onRead(yarp::rosmsg::sensor_msgs::JointState& v)
{
    yarp::os::Subscriber<yarp::rosmsg::sensor_msgs::JointState>::onRead(v);

    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_connector)
        m_connector->onRead(v);
}

void JointStateConnector::onRead(yarp::rosmsg::sensor_msgs::JointState &v)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_connected)
            return;

        for (size_t i=0; i < v.name.size(); i++)
        {

            auto jointIndex_it = m_nameToIndexMap.find(v.name[i]);
            if (jointIndex_it != m_nameToIndexMap.end())
            {
                m_jointsInRad(jointIndex_it->second) = v.position[i];
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_callbackMutex);

        if (!m_connected)
            return;

        //Update the joint values
        if (m_callback)
        {
            m_callback();
        }
    }
}

void JointStateConnector::setCallback(std::function<void ()> callback)
{
    std::lock_guard<std::mutex> lock(m_callbackMutex);

    m_callback = callback;
}

bool JointStateConnector::configure(const yarp::os::Searchable &inputConf, const iDynTree::Model &fullModel, std::shared_ptr<BasicInfo> basicInfo)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_basicInfo = basicInfo;

    if (!m_basicInfo)
    {
        yError() << "Basic info pointer not set.";
        return false;
    }

    m_namePrefix = inputConf.check("name-prefix",Value("")).asString();
    m_jointStatesTopicName = inputConf.check("jointstates-topic",Value("/joint_states")).asString();

    if (!getJointNamesFromModel(inputConf, fullModel))
    {
        yError() << "Failed to get the list of joints.";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_basicInfo->mutex);
        for (size_t i = 0; i < m_basicInfo->jointList.size(); ++i)
        {
            m_nameToIndexMap[m_basicInfo->jointList[i]] = i;
        }
    }

    return true;
}

bool JointStateConnector::connectToRobot()
{
    this->close();
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_namePrefix.empty()) {
            m_rosNode = std::make_unique<yarp::os::Node>("/"+m_namePrefix+"/yarprobotstatepublisher");
        }
        else {
            m_rosNode = std::make_unique<yarp::os::Node>("/yarprobotstatepublisher");
        }
        // Setup the topic and configureisValid the onRead callback
        m_subscriber = std::make_unique<JointStateSubscriber>();
        m_subscriber->attach(this);

        if (!m_subscriber->topic(m_jointStatesTopicName))
        {
            return false;
        }
        m_subscriber->useCallback();
    }

    m_connected = true;
    return true;
}

bool JointStateConnector::getJointValues(iDynTree::VectorDynSize &jointValuesInRad)
{
    if (!m_connected)
        return false;

    std::lock_guard<std::mutex> lock(m_mutex);
    jointValuesInRad = m_jointsInRad;

    return true;
}

void JointStateConnector::close()
{
    m_connected = false;
    {
        std::lock_guard<std::mutex> lockCallback(m_callbackMutex);
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_subscriber)
        {
            m_subscriber->interrupt();
            m_subscriber->disableCallback();
            m_subscriber->close();
        }
        m_rosNode = nullptr;
        m_subscriber = nullptr;
    }
}

/************************************************************/
bool YARPRobotStatePublisherModule::configureTransformServer(const string &name, const yarp::os::Searchable &rf)
{

    string namePrefix = rf.check("name-prefix",Value("")).asString();

    Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", "transformClient");
    if (!namePrefix.empty()) {
        pTransformclient_cfg.put("local", "/"+namePrefix+"/"+name+"/transformClient");
    }
    else pTransformclient_cfg.put("local", "/"+name+"/transformClient");

    pTransformclient_cfg.put("remote", "/transformServer");

    m_tfPrefix = rf.check("tf-prefix",Value("")).asString();

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


/************************************************************/
bool YARPRobotStatePublisherModule::configure(ResourceFinder &rf)
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

    string modelFileName=rf.check("model",Value("model.urdf")).asString();
    // Open the model
    string pathToModel=rf.findFileByName(modelFileName);

    if (pathToModel == "")
    {
        yError() << "Failed to find" << modelFileName;
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    bool ok = modelLoader.loadModelFromFile(pathToModel);

    m_connector = std::make_shared<JointStateConnector>();
    if (!m_connector->configure(rf, modelLoader.model(), basicInfo))
    {
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
    }

    m_period=rf.check("period",Value(0.010)).asFloat64();
    m_treeType=rf.check("tree-type", Value("SHALLOW")).asString();
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

    m_connector->setCallback([this](){this->onReadCallback();});
    m_useCallback = true;

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

    // Disconnect the topic subscriber
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
        yInfoThrottle(5.0) << "YARPRobotStatePublisherModule running happily";
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

