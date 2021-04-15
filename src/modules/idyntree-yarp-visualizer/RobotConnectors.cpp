#include <yarp/os/LogStream.h>
#include <iDynTree/Core/Utils.h>
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
                yError() << "The value in position " << i << " (0-based) of controlBoards is not a string.";
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
                yError() << "The value in position " << i << " (0-based) of joints is not a string.";
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
    std::lock_guard<std::mutex> lock(m_mutex);
    m_robotDevice.close();
    m_encodersInterface = nullptr;
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
        yError() << "The list provided after connectToStateExt is supposed to have an even number of elements. It is supposed to be a sequence of a string (the name of the control board), followed by the list of joints in the control board.";
        return false;
    }

    for (size_t i = 0; i < cbJointList->size(); i += 2)
    {
        yarp::os::Value& cbName = cbJointList->get(i);
        if (!cbName.isString())
        {
            yError() << "The element in position " << i << " (0-based) of the connectToStateExt list is supposed to be a string, namely the name of the control board.";
            return false;
        }

        yarp::os::Value& jointListValue = cbJointList->get(i+1);
        if (!jointListValue.isList())
        {
            yError() << "The element in position " << i+1 << " (0-based) of the connectToStateExt list is supposed to be a list, namely the list of joints of the specific control board.";
            return false;
        }

        m_cb_jointsMap.push_back(std::make_pair(cbName.asString(), std::vector<JointInfo>()));

        std::vector<JointInfo>& jointVector = m_cb_jointsMap.back().second;

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
                    yError() << "The element at position " << j << " (0-based) of the control board \""
                             << m_cb_jointsMap.back().first << "\" is malformed. It has "
                             << jointBottle->size() << ", but it is supposed to have 2.";
                    return false;
                }

                yarp::os::Value& jointNameValue = jointBottle->get(0);

                if (!jointNameValue.isString())
                {
                    yError() << "The element at position " << j << " (0-based) of the control board \""
                             << m_cb_jointsMap.back().first << "\" is malformed."
                             << " The first element is supposed to be the name of the joint, but it is not a string.";
                    return false;
                }

                yarp::os::Value& joinTypeValue = jointBottle->get(1);

                if (!joinTypeValue.isString())
                {
                    yError() << "The element at position " << j << " (0-based) of the control board \""
                             << m_cb_jointsMap.back().first << "\" is malformed."
                             << " The second element is supposed to be the type of the joint, but it is not a string.";
                    return false;
                }

                JointInfo newJoint;
                newJoint.name = joinTypeValue.asString();

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
                    yError() << "The element at position " << j << " (0-based) of the control board \""
                             << m_cb_jointsMap.back().first << "\" is malformed."
                             << " The second element is supposed to be the type of the joint, but the value "
                             << joinTypeValue.asString() << " is not recognized. Supported values are \"p\" or \"prismatic\" "
                             << "for prismatic joints, and \"r\" and \"revolute\" for revolute joints.";
                    return false;
                }

                jointVector.push_back(newJoint);
            }
            else
            {
                yError() << "The element at position " << j << " (0-based) of the control board \""
                         << m_cb_jointsMap.back().first << "\" is malformed."
                         <<" It is supposed to be either a string or a list of two elements (the joint name and the joint type).";
                return false;
            }

            jointNamesVector.push_back(jointVector.back().name);
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
    std::lock_guard<std::mutex> lock(m_mutex);

    return true;
}

bool StateExtConnector::getJointValues(iDynTree::VectorDynSize &jointValuesInRad)
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

void StateExtConnector::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_robotDevice.close();
    m_encodersInterface = nullptr;
}

}
