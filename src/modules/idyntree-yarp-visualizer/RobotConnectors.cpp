#include <yarp/os/LogStream.h>
#include <iDynTree/Core/Utils.h>
#include <yarp/dev/IAxisInfo.h>
#include "RobotConnectors.h"
#include <algorithm>

namespace idyntree_yarp_tools {

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

bool RemapperConnector::getJointNamesFromModel(const yarp::os::Searchable &inputConf, const iDynTree::Model &model)
{
    std::lock_guard<std::mutex> lock(m_basicInfo->mutex);

    m_basicInfo->jointList.clear();

    yarp::os::Value jointsValue = inputConf.find("joints");

    if (jointsValue.isNull())
    {
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

    return true;
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
        std::lock_guard<std::mutex> lock(m_basicInfo->mutex);
        m_robotDevice.close();
        m_encodersInterface = nullptr;

        yarp::os::Property remapperOptions;
        remapperOptions.put("device", "remotecontrolboardremapper");
        yarp::os::Bottle axesNames;
        yarp::os::Bottle & axesList = axesNames.addList();
        for (auto& joint : m_basicInfo->jointList)
        {
            axesList.addString(joint);
        }
        remapperOptions.put("axesNames",axesNames.get(0));

        yarp::os::Bottle remoteControlBoards;
        yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
        for (auto& cb : m_controlBoards)
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
    }

    m_connected = true;
    return true;
}

bool RemapperConnector::getJointValues(iDynTree::VectorDynSize &jointValuesInRad)
{
    if (m_connected)
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_encodersInterface->getEncoders(m_jointsInDeg.data()))
        {
            for (size_t i = 0; i < m_jointsInDeg.size(); ++i)
            {
                jointValuesInRad(i) = iDynTree::deg2rad(m_jointsInDeg(i));
            }
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

bool StateExtConnector::getAxesDescription(const yarp::os::Value &inputValue)
{
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
                 << "It is supposed to be a sequence of a list containing the name of the control board (and eventuually the number of joints to consider),"
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
                if (!cbNameList->get(1).isInt())
                {
                    yError() << "The element in position" << i << "(0-based) of the connectToStateExt list is a malformed list. The second element is supposed to be int defining the number of considered joints.";
                    return false;
                }

                m_cb_jointsMap.back().jointsToConsider = cbNameList->get(1).asInt();
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

    if (!getAxesDescription(inputConf.find("connectToStateExt")))
    {
        yError() << "Failed to read the connectToStateExt parameter.";
        return false;
    }

    return true;
}

bool StateExtConnector::usingStateExt(const yarp::os::Searchable &inputConf)
{
    return inputConf.check("connectToStateExt");
}

bool StateExtConnector::connectToRobot()
{
    m_connected = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);

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
                    axis.addVocab(yarp::dev::VOCAB_JOINTTYPE_PRISMATIC);
                    break;

                case JointType::REVOLUTE:
                    axis.addVocab(yarp::dev::VOCAB_JOINTTYPE_REVOLUTE);
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
                        jointValuesInRad(currentPosition) = iDynTree::deg2rad(m_encodersInterfaces[cb].jointsBuffer(i));
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
    device->close();
    delete device;
}

}
