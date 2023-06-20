/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef  IDYNTREE_YARP_ROBOTCONNECTORS_H
#define  IDYNTREE_YARP_ROBOTCONNECTORS_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/rosmsg/sensor_msgs/JointState.h>
#include <yarp/os/Node.h>
#include <yarp/os/Subscriber.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/Model.h>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <functional>
#include <unordered_map>
#include <map>



namespace idyntree_yarp_tools {

enum class ConnectionType
{
    REMAPPER,
    STATE_EXT,
    JOINT_STATE
};

struct BasicInfo
{
    std::string name;

    std::vector<std::string> jointList;

    std::string robotPrefix;

    std::mutex mutex;
};

/************************************************************/

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

    static ConnectionType RequestedType(const yarp::os::Searchable &inputConf, ConnectionType defaultType);

    virtual bool connectToRobot() = 0;

    virtual bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad) = 0;

    virtual void close() = 0;
};

/************************************************************/

class RemapperConnector : public BasicConnector
{
    struct Joint {
        std::string name;
        size_t desiredPosition;
    };

    std::vector<std::string> m_controlBoards;
    std::vector<std::string> m_availableControlBoards;
    std::vector<Joint> m_availableJoints;
    iDynTree::VectorDynSize m_availableJointsInDeg;
    yarp::dev::PolyDriver m_robotDevice;
    yarp::dev::IEncodersTimed *m_encodersInterface{nullptr};

    bool getOrGuessControlBoardsFromFile(const yarp::os::Searchable &inputConf);

    bool addJointsFromBoard(const std::string& name,
                            const std::string& robot,
                            const std::string& controlBoard,
                            const std::unordered_map<std::string, size_t> &desiredJointsMap);

    void getAvailableJoints(const std::string& name, const std::string& robot, const std::vector<std::string> &desiredJoints);

    void fillDesiredJointsInDeg();

public:

    bool configure(const yarp::os::Searchable &inputConf, const iDynTree::Model& fullModel, std::shared_ptr<BasicInfo> basicInfo);

    virtual bool connectToRobot() override;

    virtual bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad) override;

    virtual void close() override;
};

/************************************************************/

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

/************************************************************/

class JointStateConnector: public BasicConnector
{
private:

    class JointStateSubscriber: public yarp::os::Subscriber<yarp::rosmsg::sensor_msgs::JointState>
    {
    private:
        // Mutex protecting the method across the different threads
        std::mutex m_mutex;

        JointStateConnector* m_connector{nullptr};

    public:
        JointStateSubscriber();

        ~JointStateSubscriber();

        JointStateSubscriber(const JointStateSubscriber&) = delete;
        JointStateSubscriber(JointStateSubscriber&&) = delete;
        JointStateSubscriber& operator=(const JointStateSubscriber&) = delete;
        JointStateSubscriber& operator=(JointStateSubscriber&&) = delete;

        void attach(JointStateConnector* connector);

        virtual void onRead(yarp::rosmsg::sensor_msgs::JointState &v) override;
    };

    // Mutex protecting the method across the different threads
    std::mutex m_mutex;
    std::mutex m_callbackMutex;

    std::function<void()> m_callback;

    // /JointState topic scruscriber
    std::unique_ptr<yarp::os::Node> m_rosNode{nullptr};
    std::unique_ptr<JointStateSubscriber> m_subscriber{nullptr};
    std::string m_jointStatesTopicName;
    std::string m_namePrefix;
    std::unordered_map<std::string, size_t> m_nameToIndexMap;
    std::atomic<bool> m_connected{false};

    void onRead(yarp::rosmsg::sensor_msgs::JointState &v);

public:
    void setCallback(std::function<void()> callback);

    bool configure(const yarp::os::Searchable &inputConf, const iDynTree::Model& fullModel, std::shared_ptr<BasicInfo> basicInfo);

    virtual bool connectToRobot() override;

    virtual bool getJointValues(iDynTree::VectorDynSize& jointValuesInRad) override;

    virtual void close() override;
};

/************************************************************/

}

#endif //  IDYNTREE_YARP_ROBOTCONNECTORS_H
