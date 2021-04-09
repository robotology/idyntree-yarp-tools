#ifndef IDYNTREE_YARP_VISUALIZER_H
#define IDYNTREE_YARP_VISUALIZER_H

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <Utilities.h>
#include <mutex>
#include <thrifts/VisualizerCommands.h>


namespace idyntree_yarp_tools {

class Visualizer : public VisualizerCommands
{
    std::vector<std::string> jointList =
    {"neck_pitch", "neck_roll", "neck_yaw",
     "torso_pitch", "torso_roll", "torso_yaw",
     "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
     "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw",
     "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll",
     "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    std::vector<std::string> controlBoards = {"head", "torso", "left_arm", "right_arm", "left_leg", "right_leg"};

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IEncodersTimed *encodersInterface{nullptr};

    std::string robotPrefix = "icubSim";

    bool connectToRobot = false;
    bool useNetwork = true;

    iDynTree::ModelLoader modelLoader;

    iDynTree::Visualizer viz;
    iDynTree::ITexture* textureInterface{nullptr};
    iDynTree::VisualizerOptions options, textureOptions;

    std::vector<iDynTree::PixelViz> pixels;

    yarp::sig::FlexImage image;
    yarp::os::BufferedPort<yarp::sig::FlexImage> imagePort;

    std::chrono::steady_clock::time_point now, lastSent, lastViz;

    unsigned int desiredFPS = 30;
    unsigned int maxVizFPS = 65;
    bool mirrorImage = false;

    long minimumMicroSec;
    long minimumMicroSecViz;

    iDynTree::Transform wHb;
    iDynTree::VectorDynSize joints;
    iDynTree::VectorDynSize jointsInDeg;

    std::atomic<bool> m_isClosing{false};

    yarp::os::Port m_rpcPort;

    std::mutex m_mutex;

public:
    bool configure();

    int run();

    bool update();

    void close();

    void closeSignalHandler();

    /**
     * Set the base position.
     * The values are expected in meters.
     * @return true/false in case of success/failure;
     */
    virtual bool setBasePosition(const double x, const double y, const double z) override;

    /**
     * Set the base rotation.
     * The values are expected in degrees.
     * @return true/false in case of success/failure;
     */
    virtual bool setBaseRotation(const double roll, const double pitch, const double yaw) override;

    /**
     * Set the base pose.
     * The x, y, and z values are expected in meters
     * The roll, pitch, and yaw avalues are expected in degrees.
     * @return true/false in case of success/failure;
     */
    virtual bool setBasePose(const double x, const double y, const double z, const double roll, const double pitch, const double yaw) override;
};

} //namespace idyntree_yarp_tools

#endif // IDYNTREE_YARP_VISUALIZER_H
