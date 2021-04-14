#ifndef IDYNTREE_YARP_VISUALIZER_H
#define IDYNTREE_YARP_VISUALIZER_H

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <thrifts/VisualizerCommands.h>
#include <string>
#include <Utilities.h>
#include "RobotConnectors.h"


namespace idyntree_yarp_tools {

class Visualizer : public VisualizerCommands
{

    enum class ConnectionType
    {
        REMAPPER,
        STATE_EXT
    };

    std::shared_ptr<BasicInfo> m_basicInfo;

    iDynTree::ModelLoader m_modelLoader;

    iDynTree::Visualizer m_viz;
    iDynTree::ITexture* m_textureInterface{nullptr};

    std::vector<iDynTree::PixelViz> m_pixels;

    yarp::sig::FlexImage m_image;
    yarp::os::BufferedPort<yarp::sig::FlexImage> m_imagePort;

    std::chrono::steady_clock::time_point m_now, m_lastSent, m_lastViz;

    unsigned int m_desiredFPS;
    unsigned int m_maxVizFPS;
    bool m_mirrorImage;

    long m_minimumMicroSec;
    long m_minimumMicroSecViz;

    iDynTree::Transform m_wHb;
    iDynTree::VectorDynSize m_joints;

    std::atomic<bool> m_isClosing{false};
    std::atomic<bool> m_connectedToTheRobot{false};
    std::atomic<bool> m_offline{false};

    std::atomic<ConnectionType> m_connectionType{ConnectionType::REMAPPER};
    RemapperConnector m_remapperConnector;

    yarp::os::Port m_rpcPort;

    std::mutex m_mutex;

    bool connectToTheRobot();

    bool setVizOptionsFromConfig(const yarp::os::Searchable &inputConf, iDynTree::VisualizerOptions &output, unsigned int& fps);

    bool setVizEnvironmentFromConfig(const yarp::os::Searchable &inputConf, iDynTree::IEnvironment& environment);

    bool setVizCameraFromConfig(const yarp::os::Searchable &inputConf, iDynTree::ICamera& camera);

    bool getOrGuessBasicInfo(const yarp::os::Searchable &inputConf, const iDynTree::Model& model);

    void updateJointValues();

public:

    bool configure(const yarp::os::ResourceFinder& rf);

    bool neededHelp(const yarp::os::ResourceFinder& rf);

    int run();

    bool update();

    void close();

    void closeSignalHandler();

    /**
     * Set the base position.
     * The values are expected in meters.
     * @return true/false in case of success/failure.
     */
    virtual bool setBasePosition(const double x, const double y, const double z) override;

    /**
     * Set the base rotation.
     * The values are expected in degrees.
     * @return true/false in case of success/failure.
     */
    virtual bool setBaseRotation(const double roll, const double pitch, const double yaw) override;

    /**
     * Set the base pose.
     * The x, y, and z values are expected in meters
     * The roll, pitch, and yaw avalues are expected in degrees.
     * @return true/false in case of success/failure.
     */
    virtual bool setBasePose(const double x, const double y, const double z, const double roll, const double pitch, const double yaw) override;

    /**
     * Attempt to reconnect to the robot.
     * @return true/false in case of success/failure.
     */
    virtual bool reconnectToRobot() override;

    /**
     * Get the camera position
     * @return A vector of 3 elements with the camera position.
     */
    virtual std::vector<double> getCameraPosition() override;

    /**
     * Get the camera target
     * @return A vector of 3 elements with the camera target.
     */
    virtual std::vector<double> getCameraTarget() override;
};

} //namespace idyntree_yarp_tools

#endif // IDYNTREE_YARP_VISUALIZER_H
