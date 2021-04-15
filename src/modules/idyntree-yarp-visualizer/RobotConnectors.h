#ifndef  IDYNTREE_YARP_ROBOTCONNECTORS_H
#define  IDYNTREE_YARP_ROBOTCONNECTORS_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/Model.h>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <map>
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

    bool getOrGuessControlBoardsFromFile(const yarp::os::Searchable &inputConf);

    bool getJointNamesFromModel(const yarp::os::Searchable &inputConf, const iDynTree::Model& model);


public:

    bool configure(const yarp::os::Searchable &inputConf, const iDynTree::Model& fullModel, std::shared_ptr<BasicInfo> basicInfo);

    bool connectToRobot();

    bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad);

    void close();
};

class StateExtConnector
{
    enum class JointType
    {
        REVOLUTE,
        PRISMATIC
    };

    struct JointInfo
    {
        std::string name;
        JointType type;
    };

    struct EncodersInterface
    {
        yarp::dev::PolyDriver* device;
        yarp::dev::IEncodersTimed* encoders{nullptr};
        iDynTree::VectorDynSize jointsBuffer;

        ~EncodersInterface();
    };

    std::vector<std::pair<std::string, std::vector<JointInfo>>> m_cb_jointsMap;
    std::vector<EncodersInterface> m_encodersInterfaces;
    std::atomic<bool> m_connected{false};
    std::shared_ptr<BasicInfo> m_basicInfo;
    std::mutex m_mutex;

    bool getAxesDescription(const yarp::os::Value &inputValue);

public:

    bool configure(const yarp::os::Searchable &inputConf, std::shared_ptr<BasicInfo> basicInfo);

    bool usingStateExt(const yarp::os::Searchable &inputConf);

    bool connectToRobot();

    bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad);

    void close();
};


}


#endif //  IDYNTREE_YARP_ROBOTCONNECTORS_H
