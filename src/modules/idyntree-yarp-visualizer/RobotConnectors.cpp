#include <yarp/os/LogStream.h>
#include <iDynTree/Core/Utils.h>
#include "RobotConnectors.h"

namespace idyntree_yarp_tools {

bool RemapperConnector::getorGuessControlBoardsFromFile(const yarp::os::Searchable &inputConf)
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

bool RemapperConnector::configure(const yarp::os::Searchable &inputConf, std::shared_ptr<BasicInfo> basicInfo)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!getorGuessControlBoardsFromFile(inputConf))
    {
        return false;
    }

    m_basicInfo = basicInfo;

    {
        std::lock_guard<std::mutex>(m_basicInfo->mutex);

        m_jointsInDeg.resize(m_basicInfo->jointList.size());
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

}
