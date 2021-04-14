#ifndef  IDYNTREE_YARP_ROBOTCONNECTORS_H
#define  IDYNTREE_YARP_ROBOTCONNECTORS_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <memory>


namespace idyntree_yarp_tools {

struct BasicInfo
{
    std::string name;

    std::vector<std::string> jointList;

    std::string robotPrefix;

    std::mutex mutex;
};

class RemapperConnector
{

    std::vector<std::string> m_controlBoards;
    yarp::dev::PolyDriver m_robotDevice;
    yarp::dev::IEncodersTimed *m_encodersInterface{nullptr};
    std::atomic<bool> m_connected{false};
    iDynTree::VectorDynSize m_jointsInDeg;
    std::shared_ptr<BasicInfo> m_basicInfo;
    std::mutex m_mutex;

    bool getorGuessControlBoardsFromFile(const yarp::os::Searchable &inputConf);

public:

    bool configure(const yarp::os::Searchable &inputConf, std::shared_ptr<BasicInfo> basicInfo);

    bool connectToRobot();

    bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad);

    void close();
};


}


#endif //  IDYNTREE_YARP_ROBOTCONNECTORS_H
