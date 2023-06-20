/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <yarp/os/LogStream.h>
#include <iDynTree/Core/Utils.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IPositionControl.h>
#include "RobotConnectors.h"
#include <algorithm>

YARP_DECLARE_PLUGINS(ReadOnlyRemoteControlBoardLib);

using namespace idyntree_yarp_tools;

/************************************************************/

bool BasicConnector::getJointNamesFromModel(const yarp::os::Searchable &inputConf, const iDynTree::Model &model)
{
    if (!m_basicInfo)
    {
        yError() << "Basic info pointer not set.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_basicInfo->mutex);

    m_basicInfo->jointList.clear();

    yarp::os::Value jointsValue = inputConf.find("joints");

    if (jointsValue.isNull())
    {
        yInfo() << "Retrieving the list of joints from the model.";

        for (size_t i = 0; i < model.getNrOfJoints(); ++i)
        {
            if (model.getJoint(i)->getNrOfDOFs() == 1)
            {
                m_basicInfo->jointList.push_back(model.getJointName(i)); //automatically select all the joint in the model that have 1DOF
            }
        }
        if (m_basicInfo->jointList.size() != model.getNrOfPosCoords())
        {
            yError() << "The model contains joints with more than 1DOF. This is not yet supported.";
            return false;
        }
    }
    else
    {
        if (!jointsValue.isList())
        {
            yError() << "joints is specified, but it is not a list.";
            return false;
        }

        yarp::os::Bottle* jointList = jointsValue.asList();

        for (size_t i = 0; i < jointList->size(); ++i)
        {
            yarp::os::Value jvalue = jointList->get(i);
            if (!jvalue.isString())
            {
                yError() << "The value in position" << i << "(0-based) of joints is not a string.";
                return false;
            }
            m_basicInfo->jointList.push_back(jvalue.asString());
        }
    }

    m_jointsInDeg.resize(m_basicInfo->jointList.size());
    m_jointsInDeg.zero();

    m_jointsInRad = m_jointsInDeg;

    return true;
}

void BasicConnector::fillJointValuesInRad()
{
    for (size_t i = 0; i < m_jointsInDeg.size(); ++i)
    {
        m_jointsInRad(i) = iDynTree::deg2rad(m_jointsInDeg(i));
    }
}

ConnectionType BasicConnector::RequestedType(const yarp::os::Searchable &inputConf, ConnectionType defaultType)
{
    if (inputConf.check("connectToStateExt"))
    {
        return ConnectionType::STATE_EXT;
    }

    if (inputConf.check("connectToControlBoards"))
    {
        return ConnectionType::REMAPPER;
    }

    if (inputConf.check("connectToJointState"))
    {
        return ConnectionType::JOINT_STATE;
    }

    return defaultType;
}

/************************************************************/

bool RemapperConnector::getOrGuessControlBoardsFromFile(const yarp::os::Searchable &inputConf)
{
    m_controlBoards.clear();
    yarp::os::Value controlBoardsValue = inputConf.find("controlboards");

    if (controlBoardsValue.isNull())
    {
        m_controlBoards = {"head", "torso", "left_arm", "right_arm", "left_leg", "right_leg"}; //assumes these as controlboards
    }
    else
    {
        if (!controlBoardsValue.isList())
        {
            yError() << "controlBoards is specified, but it is not a list.";
            return false;
        }

        yarp::os::Bottle* controlBoardsList = controlBoardsValue.asList();

        for (size_t i = 0; i < controlBoardsList->size(); ++i)
        {
            yarp::os::Value cbValue = controlBoardsList->get(i);
            if (!cbValue.isString())
            {
                yError() << "The value in position" << i << "(0-based) of controlBoards is not a string.";
                return false;
            }
            m_controlBoards.push_back(cbValue.asString());
        }
    }

    return true;
}

bool RemapperConnector::addJointsFromBoard(const std::string &name,
                                           const std::string &robot,
                                           const std::string &controlBoard,
                                           const std::unordered_map<std::string, size_t>& desiredJointsMap)
{
    yarp::os::Property rcb_conf{
        {"device", yarp::os::Value("remote_controlboard")},
        {"local", yarp::os::Value("/" + name + "/" + controlBoard + "/remoteControlBoard")},
        {"remote", yarp::os::Value("/" + robot + "/" + controlBoard)},
        {"part", yarp::os::Value(controlBoard)}};

    yarp::dev::PolyDriver m_driver;

    if (!m_driver.open(rcb_conf))
    {
        return false;
    }

    yarp::dev::IAxisInfo* axisInfo{nullptr};
    yarp::dev::IPositionControl* pos{nullptr};

    if (!m_driver.view(pos) || !pos)
    {
        return false;
    }

    if (!m_driver.view(axisInfo) || !axisInfo)
    {
        return false;
    }

    int nAxes = 0;

    if (!pos->getAxes(&nAxes))
    {
        return false;
    }

    for (size_t i = 0; i < nAxes; ++i)
    {
        std::string name;
        if (axisInfo->getAxisName(i,name))
        {
            auto it = desiredJointsMap.find(name);
            if (it != desiredJointsMap.end())
            {
                m_availableJoints.push_back({name, it->second});
            }
        }
    }

    return true;
}

void RemapperConnector::getAvailableJoints(const std::string &name,
                                           const std::string &robot,
                                           const std::vector<std::string>& desiredJoints)
{
    m_availableControlBoards.clear();
    m_availableJoints.clear();

    std::unordered_map<std::string, size_t> desiredJointsMap;

    for (size_t i = 0; i < m_basicInfo->jointList.size(); ++i)
    {
        desiredJointsMap[m_basicInfo->jointList[i]] = i;
    }

    for (const std::string& cb : m_controlBoards)
    {
        if (addJointsFromBoard(name, robot, cb, desiredJointsMap))
        {
            m_availableControlBoards.push_back(cb);
        }
    }

    m_availableJointsInDeg.resize(m_availableJoints.size());
    m_availableJointsInDeg.zero();
}

void RemapperConnector::fillDesiredJointsInDeg()
{
    for (size_t i = 0; i < m_availableJoints.size(); ++i)
    {
        m_jointsInDeg[m_availableJoints[i].desiredPosition] = m_availableJointsInDeg[i];
    }
}

bool RemapperConnector::configure(const yarp::os::Searchable &inputConf, const iDynTree::Model &fullModel, std::shared_ptr<BasicInfo> basicInfo)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_basicInfo = basicInfo;

    if (!getOrGuessControlBoardsFromFile(inputConf))
    {
        yError() << "Failed to get the controlboards list.";
        return false;
    }

    if (!getJointNamesFromModel(inputConf, fullModel))
    {
        yError() << "Failed to get the list of joints.";
        return false;
    }

    return true;
}

bool RemapperConnector::connectToRobot()
{
    m_connected = false;

    std::lock_guard<std::mutex> lock(m_mutex);
    {
        if (!m_basicInfo)
        {
            yError() << "Basic info pointer not set.";
            return false;
        }

        std::lock_guard<std::mutex> lock(m_basicInfo->mutex);

        getAvailableJoints(m_basicInfo->name, m_basicInfo->robotPrefix, m_basicInfo->jointList);

        //First make sure to reset the device in case it was already opened.
        m_robotDevice.close();
        m_encodersInterface = nullptr;

        if (m_availableControlBoards.size() == 0)
        {
            yError() << "No available boards.";
            return false;
        }

        if (m_availableJoints.size() == 0)
        {
            yError() << "No available joints.";
            return false;
        }

        yarp::os::Property remapperOptions;
        remapperOptions.put("device", "remotecontrolboardremapper");
        yarp::os::Bottle axesNames;
        yarp::os::Bottle & axesList = axesNames.addList();
        for (auto& joint : m_availableJoints)
        {
            axesList.addString(joint.name);
        }
        remapperOptions.put("axesNames",axesNames.get(0));

        yarp::os::Bottle remoteControlBoards;
        yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
        for (auto& cb : m_availableControlBoards)
        {
            remoteControlBoardsList.addString("/" + m_basicInfo->robotPrefix + "/" + cb);
        }
        remapperOptions.put("remoteControlBoards",remoteControlBoards.get(0));
        remapperOptions.put("localPortPrefix", "/" + m_basicInfo->name +"/remoteControlBoard:i");

        if(!m_robotDevice.open(remapperOptions))
        {
            yError() << "Failed to open remote control board remapper.";
            return false;
        }

        if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
        {
            yError() << "Failed to view encoder interface.";

            return false;
        }

        std::stringstream boardsInfo;

        boardsInfo << "Connected to the following boards:" <<std::endl;
        for (const std::string& cb : m_availableControlBoards)
        {
            boardsInfo << "- " << cb << std::endl;
        }

        yInfo() << boardsInfo.str();

        std::stringstream jointsInfo;
        jointsInfo << "Connected to the following joints:" <<std::endl;
        for (const Joint& joint : m_availableJoints)
        {
            jointsInfo << "- " << joint.name << std::endl;
        }

        yInfo() << jointsInfo.str();

    }

    m_connected = true;
    return true;
}

bool RemapperConnector::getJointValues(iDynTree::VectorDynSize &jointValuesInRad)
{
    if (m_connected)
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_encodersInterface->getEncoders(m_availableJointsInDeg.data()))
        {
            fillDesiredJointsInDeg();
            fillJointValuesInRad();
            jointValuesInRad = m_jointsInRad;
        }
    }
    return true;
}

void RemapperConnector::close()
{
    m_connected = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_robotDevice.close();
        m_encodersInterface = nullptr;
    }
}

/************************************************************/

bool StateExtConnector::getAxesDescription(const yarp::os::Value &inputValue)
{
    if (!m_basicInfo)
    {
        yError() << "Basic info pointer not set.";
        return false;
    }

    m_cb_jointsMap.clear();
    std::vector<std::string> jointNamesVector;

    if (!inputValue.isList())
    {
        yError() << "connectToStateExt is specified, but the input is not a list.";
        return false;
    }

    yarp::os::Bottle* cbJointList = inputValue.asList();

    if (cbJointList->size() %2 != 0)
    {
        yError() << "The list provided after connectToStateExt is supposed to have an even number of elements."
                 << "It needs to be a sequence of a list containing the name of the control board (and eventually the number of joints to consider),"
                     "followed by the full list of joints in the control board.";
        return false;
    }

    for (size_t i = 0; i < cbJointList->size(); i += 2)
    {
        m_cb_jointsMap.emplace_back();
        m_cb_jointsMap.back().jointsToConsider = -1;
        std::vector<JointInfo>& jointVector = m_cb_jointsMap.back().joints;

        yarp::os::Value& cbName = cbJointList->get(i);
        if (cbName.isString())
        {
            m_cb_jointsMap.back().name = cbName.asString();

        }
        else if (cbName.isList())
        {
            yarp::os::Bottle* cbNameList = cbName.asList();
            if (cbNameList->size() != 2 && cbNameList->size() != 1)
            {
                yError() << "The element in position" << i << "(0-based) of the connectToStateExt list is a malformed list. It is supposed to be one or two dimensional,"
                            " containing the name of the board and, optionally the number of considered joints";
                return false;
            }

            if (!cbNameList->get(0).isString())
            {
                yError() << "The element in position" << i << "(0-based) of the connectToStateExt list is a malformed list. The first element is supposed to be a string containing the name of the control board.";
                return false;
            }

            m_cb_jointsMap.back().name = cbNameList->get(0).asString();

            if (cbNameList->size() == 2)
            {
                if (!cbNameList->get(1).isInt32())
                {
                    yError() << "The element in position" << i << "(0-based) of the connectToStateExt list is a malformed list. The second element is supposed to be int defining the number of considered joints.";
                    return false;
                }

                m_cb_jointsMap.back().jointsToConsider = cbNameList->get(1).asInt32();
            }
        }
        else
        {
            yError() << "The element in position" << i << "(0-based) of the connectToStateExt list is supposed to be a string or a list, containing the name of the board and, optionally the number of considered joints.";
            return false;
        }

        yarp::os::Value& jointListValue = cbJointList->get(i+1);
        if (!jointListValue.isList())
        {
            yError() << "The element in position" << i+1 << "(0-based) of the connectToStateExt list is supposed to be a list, namely the list of joints of the specific control board.";
            return false;
        }

        yarp::os::Bottle* jointList = jointListValue.asList();

        if (m_cb_jointsMap.back().jointsToConsider > static_cast<int>(jointList->size()))
        {
            yError() << "It has been specified to consider"<< m_cb_jointsMap.back().jointsToConsider
                     << "joints, but only" << jointList->size() << "are listed.";
            return false;
        }

        for (size_t j = 0; j < jointList->size(); ++j)
        {
            yarp::os::Value& jointValue = jointList->get(j);

            if (jointValue.isString())
            {
                jointVector.push_back({jointValue.asString(), JointType::REVOLUTE});
            }
            else if (jointValue.isList())
            {
                yarp::os::Bottle* jointBottle = jointValue.asList();

                if (jointBottle->size() != 2)
                {
                    yError() << "The element at position" << j << "(0-based) of the control board"
                             << m_cb_jointsMap.back().name << "is malformed. It has"
                             << jointBottle->size() << "but it is supposed to have 2.";
                    return false;
                }

                yarp::os::Value& jointNameValue = jointBottle->get(0);

                if (!jointNameValue.isString())
                {
                    yError() << "The element at position" << j << "(0-based) of the control board"
                             << m_cb_jointsMap.back().name << "is malformed."
                             << "The first element is supposed to be the name of the joint, but it is not a string.";
                    return false;
                }

                yarp::os::Value& joinTypeValue = jointBottle->get(1);

                if (!joinTypeValue.isString())
                {
                    yError() << "The element at position" << j << "(0-based) of the control board"
                             << m_cb_jointsMap.back().name << "is malformed."
                             << "The second element is supposed to be the type of the joint, but it is not a string.";
                    return false;
                }

                JointInfo newJoint;
                newJoint.name = jointNameValue.asString();

                std::string jointTypeString = joinTypeValue.asString();
                std::transform(jointTypeString.begin(), jointTypeString.end(), jointTypeString.begin(),
                    [](unsigned char c){ return std::tolower(c); }); //set to lowercase

                if (jointTypeString == "p" || jointTypeString == "prismatic")
                {
                    newJoint.type = JointType::PRISMATIC;
                }
                else if (jointTypeString == "r" || jointTypeString == "revolute")
                {
                    newJoint.type = JointType::REVOLUTE;
                }
                else
                {
                    yError() << "The element at position" << j << "(0-based) of the control board"
                             << m_cb_jointsMap.back().name << "is malformed."
                             << "The second element is supposed to be the type of the joint, but the value"
                             << joinTypeValue.asString() << "is not recognized. Supported values are \"p\" or \"prismatic\""
                             << "for prismatic joints, and \"r\" and \"revolute\" for revolute joints.";
                    return false;
                }

                jointVector.push_back(newJoint);
            }
            else
            {
                yError() << "The element at position" << j <<"(0-based) of the control board"
                         << m_cb_jointsMap.back().name << "is malformed."
                         <<"It is supposed to be either a string or a list of two elements (the joint name and the joint type).";
                return false;
            }

            if ((m_cb_jointsMap.back().jointsToConsider < 0) || (static_cast<int>(j) < m_cb_jointsMap.back().jointsToConsider))
            {
                jointNamesVector.push_back(jointVector.back().name);
            }
        }

        if (m_cb_jointsMap.back().jointsToConsider < 0)
        {
            m_cb_jointsMap.back().jointsToConsider = m_cb_jointsMap.back().joints.size();
        }
    }

    m_jointsInDeg.resize(jointNamesVector.size());
    m_jointsInDeg.zero();

    m_jointsInRad = m_jointsInDeg;

    {
        std::lock_guard<std::mutex> lock(m_basicInfo->mutex);
        m_basicInfo->jointList = jointNamesVector;
    }

    return true;
}

bool StateExtConnector::configure(const yarp::os::Searchable &inputConf, std::shared_ptr<BasicInfo> basicInfo)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_basicInfo = basicInfo;

    if (!m_basicInfo)
    {
        yError() << "Basic info pointer not set.";
        return false;
    }

    yarp::os::Value confValue = inputConf.find("connectToStateExt");

    if (confValue.isString() && confValue.asString() == "default")
    {
        std::string robotLocalName;
        {
            std::lock_guard<std::mutex> lock(m_basicInfo->mutex);

            robotLocalName = m_basicInfo->robotPrefix;
        }

        if (robotLocalName == "icubSim")
        {
            std::string defaultString = "(head, (neck_pitch, neck_roll, neck_yaw),"
                                        " torso, (torso_yaw, torso_pitch, torso_roll),"
                                        " left_arm, (l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw),"
                                        " right_arm, (r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw),"
                                        " left_leg, (l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll),"
                                        " right_leg, (r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll))";
            confValue.fromString(defaultString.c_str());

            yInfo() << "Using default StateExt configuration for robot icubSim. Corresponding command: \n--connectToStateExt " + defaultString + "\"";
        }
        else if (robotLocalName == "icub")
        {
            std::string defaultString = "((head, 3), (neck_pitch, neck_roll, neck_yaw, eyes_tilt, eyes_vers, eyes_verg),"
                                        " torso, (torso_roll, torso_pitch, torso_yaw),"
                                        " (left_arm, 7), (l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw,"
                                           " l_hand_finger, l_thumb_oppose, l_thumb_proximal, l_thumb_distal, l_index_proximal, l_index_distal, l_middle_proximal, l_middle_distal, l_little_fingers),"
                                        " (right_arm, 7), (r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw,"
                                           " r_hand_finger, r_thumb_oppose, r_thumb_proximal, r_thumb_distal, r_index_proximal, r_index_distal, r_middle_proximal, r_middle_distal, r_little_fingers),"
                                        " left_leg, (l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll),"
                                        " right_leg, (r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll))";
            confValue.fromString(defaultString.c_str());

            yInfo() << "Using default StateExt configuration for robot icub. Corresponding command: \n--connectToStateExt \"" + defaultString + "\"";
        }
        else
        {
            yError() << "No default connectToStateExt configuration for the robot" << robotLocalName;
            return false;
        }
    }

    if (!getAxesDescription(confValue))
    {
        yError() << "Failed to read the connectToStateExt parameter.";
        return false;
    }

    return true;
}

bool StateExtConnector::connectToRobot()
{
    m_connected = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_basicInfo)
        {
            yError() << "Basic info pointer not set.";
            return false;
        }

        YARP_REGISTER_PLUGINS(ReadOnlyRemoteControlBoardLib);

        std::string localRobotName, localName;
        {
            std::lock_guard<std::mutex> lockInfo(m_basicInfo->mutex);
            localRobotName = m_basicInfo->robotPrefix;
            localName = m_basicInfo->name;
        }

        m_encodersInterfaces.clear();
        m_encodersInterfaces.resize(m_cb_jointsMap.size());

        for (size_t i = 0; i < m_cb_jointsMap.size(); ++i)
        {

            yarp::os::Property stateExtCbOptions;
            stateExtCbOptions.put("device", "readonlyremotecontrolboard");
            stateExtCbOptions.put("remote", "/" + localRobotName + "/" + m_cb_jointsMap[i].name);
            stateExtCbOptions.put("local", "/" + localName + "/" + m_cb_jointsMap[i].name);
            stateExtCbOptions.put("carrier", "tcp");

            yarp::os::Bottle axesNames;
            yarp::os::Bottle & axesList = axesNames.addList();
            for (auto& joint : m_cb_jointsMap[i].joints)
            {
                yarp::os::Bottle& axis = axesList.addList();
                axis.addString(joint.name);

                switch (joint.type)
                {
                case JointType::PRISMATIC:
                    axis.addVocab32(yarp::dev::VOCAB_JOINTTYPE_PRISMATIC);
                    break;

                case JointType::REVOLUTE:
                    axis.addVocab32(yarp::dev::VOCAB_JOINTTYPE_REVOLUTE);
                    break;
                }

            }
            stateExtCbOptions.put("axesDescription",axesNames.get(0));

            m_encodersInterfaces[i].device = new yarp::dev::PolyDriver();

            if (!m_encodersInterfaces[i].device->open(stateExtCbOptions))
            {
                yError() << "Failed to open readonlyremotecontrolboard device for the" << m_cb_jointsMap[i].name << "control board.";
                return false;
            }

            if (!m_encodersInterfaces[i].device->view(m_encodersInterfaces[i].encoders))
            {
                yError() << "Failed to view encoder interface for the" << m_cb_jointsMap[i].name << "control board.";
                return false;
            }

            m_encodersInterfaces[i].jointsBuffer.resize(m_cb_jointsMap[i].joints.size());
            m_encodersInterfaces[i].jointsBuffer.zero();

            m_encodersInterfaces[i].jointsToConsider = m_cb_jointsMap[i].jointsToConsider;
        }
    }

    m_connected = true;

    return true;
}

bool StateExtConnector::getJointValues(iDynTree::VectorDynSize &jointValuesInRad)
{
    bool shouldClose = false;;

    if (m_connected)
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        size_t currentPosition = 0;

        for (size_t cb = 0; cb < m_encodersInterfaces.size(); ++cb)
        {
            int numberOfJoints = 0;

            if (m_encodersInterfaces[cb].encoders->getAxes(&numberOfJoints))
            {
                if (numberOfJoints != static_cast<int>(m_encodersInterfaces[cb].jointsBuffer.size()))
                {
                    yError() << "The number of joints for the control board" << m_cb_jointsMap[cb].name
                             << "appears to be" << numberOfJoints << "while it has been configured with"
                             << m_encodersInterfaces[cb].jointsBuffer.size() << "joints. Closing the connection.";
                    shouldClose = true;
                    break;
                }

                if (m_encodersInterfaces[cb].encoders->getEncoders(m_encodersInterfaces[cb].jointsBuffer.data()))
                {
                    for (size_t i = 0; i < m_encodersInterfaces[cb].jointsToConsider; ++i)
                    {
                        m_jointsInDeg(currentPosition) = m_encodersInterfaces[cb].jointsBuffer(i);
                        currentPosition++;
                    }
                }
            }
        }
    }

    if (shouldClose)
    {
        close(); //It is important to do this after unlocking the mutex to avoid deadlocks.
    }

    fillJointValuesInRad();
    jointValuesInRad = m_jointsInRad;

    return true;
}

void StateExtConnector::close()
{
    m_connected = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_encodersInterfaces.clear();
    }
}

StateExtConnector::EncodersInterface::~EncodersInterface()
{
    if (device)
    {
        device->close();
        delete device;
    }
}

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

    m_namePrefix = inputConf.check("name-prefix", yarp::os::Value("")).asString();
    m_jointStatesTopicName = inputConf.check("jointstates-topic", yarp::os::Value("/joint_states")).asString();

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

        std::string name;
        {
            std::lock_guard<std::mutex> lock(m_basicInfo->mutex);
            name = m_basicInfo->name;
        }

        if (!m_namePrefix.empty()) {
            m_rosNode = std::make_unique<yarp::os::Node>("/"+m_namePrefix+"/" +name);
        }
        else {
            m_rosNode = std::make_unique<yarp::os::Node>("/" + name);
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
