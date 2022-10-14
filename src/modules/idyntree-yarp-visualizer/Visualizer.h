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
#include <unordered_map>
#include <Utilities.h>
#include "RobotConnectors.h"
#include <thrifts/VectorsCollection.h>


namespace idyntree_yarp_tools {

class Visualizer : public VisualizerCommands
{

    std::shared_ptr<BasicInfo> m_basicInfo;

    iDynTree::ModelLoader m_modelLoader;

    iDynTree::Visualizer m_viz;
    iDynTree::ITexture* m_textureInterface{nullptr};

    std::vector<iDynTree::PixelViz> m_pixels;

    yarp::sig::FlexImage m_image;
    yarp::os::BufferedPort<yarp::sig::FlexImage> m_imagePort;

    std::chrono::steady_clock::time_point m_now, m_lastSent, m_lastViz;

    unsigned int m_desiredTextureFPS;
    unsigned int m_maxVizFPS;
    bool m_mirrorImage;
    bool m_useRGBA;

    long m_minimumMicroSec;
    long m_minimumMicroSecViz;

    iDynTree::Transform m_wHb;
    iDynTree::VectorDynSize m_joints;

    std::atomic<bool> m_isClosing{false};
    std::atomic<bool> m_connectedToTheRobot{false};
    std::atomic<bool> m_offline{false};
    std::atomic<bool> m_useWBD{true};
    std::atomic<bool> m_connectedToWBD{true};

    std::string m_remoteNextExternalWrenchesPortName;
    yarp::os::BufferedPort<VectorsCollection> m_netExternalWrenchesPort;

    struct VisualizedWrench
    {
        int arrowIndexLinear{-1};
        size_t arrowIndexAngular;
        iDynTree::Wrench scaledWrench{iDynTree::Wrench::Zero()};
        iDynTree::FrameIndex frameIndex{iDynTree::FRAME_INVALID_INDEX};
        size_t inactivityCounter{0};
        bool skip{false};
    };

    std::unordered_map<std::string, VisualizedWrench> m_netExternalWrenchesMap;
    iDynTree::ColorViz m_forcesColor, m_torquesColor;
    double m_forceMultiplier, m_torquesMultiplier;

    std::atomic<ConnectionType> m_connectionType{ConnectionType::REMAPPER};
    RemapperConnector m_remapperConnector;
    StateExtConnector m_stateExtConnector;

    yarp::os::Port m_rpcPort;

    std::mutex m_mutex;

    void connectToTheRobot();

    iDynTree::Vector4 rgbaFromConfig(const yarp::os::Searchable &inputConf, const std::string& optionName);

    bool setVizOptionsFromConfig(const yarp::os::Searchable &inputConf, iDynTree::VisualizerOptions &output, unsigned int& fps);

    bool setVizEnvironmentFromConfig(const yarp::os::Searchable &inputConf, iDynTree::IEnvironment& environment);

    bool setVizCameraFromConfig(const yarp::os::Searchable &inputConf, iDynTree::ICamera& camera);

    void updateJointValues();

    void updateWrenchesVisualization();

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
     * @return A status message. "[ok]" in case of success
     */
    virtual std::string reconnectToRobot() override;

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
