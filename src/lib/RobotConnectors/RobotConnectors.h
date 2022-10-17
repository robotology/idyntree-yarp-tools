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

enum class ConnectionType
{
    REMAPPER,
    STATE_EXT
};

struct BasicInfo
{
    std::string name;

    std::vector<std::string> jointList;

    std::string robotPrefix;

    std::mutex mutex;
};

class BasicConnector
{
protected:
    iDynTree::VectorDynSize m_jointsInDeg;
    iDynTree::VectorDynSize m_jointsInRad;
    std::shared_ptr<BasicInfo> m_basicInfo;
    std::atomic<bool> m_connected{false};
    std::mutex m_mutex;

    bool getJointNamesFromModel(const yarp::os::Searchable &inputConf, const iDynTree::Model& model);

    void fillJointValuesInRad();

public:

    static ConnectionType RequestedType(const yarp::os::Searchable &inputConf);

    virtual bool connectToRobot() = 0;

    virtual bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad) = 0;

    virtual void close() = 0;
};

class RemapperConnector : public BasicConnector
{

    std::vector<std::string> m_controlBoards;
    yarp::dev::PolyDriver m_robotDevice;
    yarp::dev::IEncodersTimed *m_encodersInterface{nullptr};

    bool getOrGuessControlBoardsFromFile(const yarp::os::Searchable &inputConf);

public:

    bool configure(const yarp::os::Searchable &inputConf, const iDynTree::Model& fullModel, std::shared_ptr<BasicInfo> basicInfo);

    virtual bool connectToRobot() override;

    virtual bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad) override;

    virtual void close() override;
};

class StateExtConnector : public BasicConnector
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

    struct ControlBoardInfo
    {
        std::string name;
        int jointsToConsider;
        std::vector<JointInfo> joints;
    };

    struct EncodersInterface
    {
        yarp::dev::PolyDriver* device{nullptr};
        yarp::dev::IEncodersTimed* encoders{nullptr};
        iDynTree::VectorDynSize jointsBuffer;
        size_t jointsToConsider;

        ~EncodersInterface();
    };

    std::vector<ControlBoardInfo> m_cb_jointsMap;
    std::vector<EncodersInterface> m_encodersInterfaces;

    bool getAxesDescription(const yarp::os::Value &inputValue);

public:

    bool configure(const yarp::os::Searchable &inputConf, std::shared_ptr<BasicInfo> basicInfo);

    virtual bool connectToRobot() override;

    virtual bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad) override;

    virtual void close() override;
};


}


#endif //  IDYNTREE_YARP_ROBOTCONNECTORS_H
