/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file robotstatepublisher.h
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#ifndef YARP_ROBOT_STATE_PUBLISHER_H
#define YARP_ROBOT_STATE_PUBLISHER_H

#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <yarp/rosmsg/sensor_msgs/JointState.h>

#include <RobotConnectors.h>

#include <mutex>
#include <memory>
#include <functional>
#include <unordered_map>

/****************************************************************/
namespace idyntree_yarp_tools {

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


/****************************************************************/
class YARPRobotStatePublisherModule : public yarp::os::RFModule
{
    std::atomic<double> m_period;
    yarp::dev::PolyDriver       m_ddtransformclient;
    yarp::dev::IFrameTransform       *m_iframetrans;

    std::string m_tfPrefix;
    std::string m_treeType;

    // Clock-related workaround
    bool m_usingNetworkClock;
    yarp::os::NetworkClock m_netClock;

    // Reduced flag option
    bool reducedModelOption;

    // Class for computing forward kinematics
   iDynTree::KinDynComputations m_kinDynComp;
   iDynTree::VectorDynSize m_jointPos;
   std::string m_baseFrameName;
   iDynTree::FrameIndex m_baseFrameIndex;
   yarp::sig::Matrix m_buf4x4;

   std::shared_ptr<JointStateConnector> m_connector{nullptr};
   std::atomic<bool> m_useCallback{false}, m_publishedOnce{false};

   // Mutex protecting the method across the different threads
   std::mutex m_mutex;

   bool configureTransformServer(const std::string& name, const yarp::os::Searchable &rf);

public:
    YARPRobotStatePublisherModule();
    virtual bool configure(yarp::os::ResourceFinder &rf) override;
    virtual bool close() override;
    virtual double getPeriod() override;
    virtual bool updateModule() override;
    void onReadCallback();
};
}

#endif

