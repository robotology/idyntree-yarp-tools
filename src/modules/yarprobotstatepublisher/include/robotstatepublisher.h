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

#include <RobotConnectors.h>

#include <mutex>
#include <memory>
#include <string>

/****************************************************************/
namespace idyntree_yarp_tools {

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
   iDynTree::VectorDynSize m_jointPos, m_jointOffsets;
   std::string m_baseFrameName;
   iDynTree::FrameIndex m_baseFrameIndex;
   yarp::sig::Matrix m_buf4x4;

   std::shared_ptr<BasicConnector> m_connector{nullptr};
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

