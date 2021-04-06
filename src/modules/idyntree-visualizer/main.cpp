/*
     * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
     *
     * Licensed under either the GNU Lesser General Public License v3.0 :
     * https://www.gnu.org/licenses/lgpl-3.0.html
     * or the GNU Lesser General Public License v2.1 :
     * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
     * at your option.
     */

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
#include <Utilities.h>

using namespace std::chrono_literals;

int main()
{
    // Listen to signals for closing in a clean way the application
    idyntree_yarp_tools::handleSigInt();

    std::vector<std::string> jointList =
    {"neck_pitch", "neck_roll", "neck_yaw",
     "torso_pitch", "torso_roll", "torso_yaw",
     "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
     "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw",
     "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll",
     "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    std::vector<std::string> controlBoards = {"head", "torso", "left_arm", "right_arm", "left_leg", "right_leg"};

    std::string robotPrefix = "icubSim";

    bool connectToRobot = false;
    bool useNetwork = true;

    iDynTree::ModelLoader modelLoader;
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
    modelLoader.loadReducedModelFromFile(pathToModel, jointList);

    iDynTree::Visualizer viz;
    iDynTree::VisualizerOptions options, textureOptions;
    //    options.winWidth = 1920;
    //    options.winHeight = 1080;

    viz.init(options);
    iDynTree::ITexture* textureInterface = viz.textures().add("AdditionalTexture", textureOptions);

    //    viz.camera().setPosition(iDynTree::Position(2.0, 0.5, 0.5));
    //    viz.camera().setTarget(iDynTree::Position(0.4, 0.0, 0.5));
    viz.camera().setPosition(iDynTree::Position(1.2, 0.0, 0.5));
    viz.camera().setTarget(iDynTree::Position(-0.15, 0.0, 0.15));
    viz.camera().animator()->enableMouseControl(true);

    double sqrt2 = std::sqrt(2.0);
    viz.enviroment().lightViz("sun").setDirection(iDynTree::Direction(0.5/sqrt2, 0, -0.5/sqrt2));
    viz.enviroment().addLight("secondSun");
    viz.enviroment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    viz.enviroment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));

    //    viz.enviroment().setFloorGridColor(iDynTree::ColorViz(0.0, 1.0, 0.0, 1.0));

    textureInterface->environment().lightViz("sun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));
    textureInterface->environment().addLight("secondSun");
    textureInterface->environment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    textureInterface->environment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));
    textureInterface->environment().setElementVisibility("floor_grid", false);
    textureInterface->environment().setElementVisibility("world_frame", false);
    textureInterface->environment().setBackgroundColor(iDynTree::ColorViz(0.0, 0.0, 0.0, 0.0));
    //    textureInterface->environment().setFloorGridColor(iDynTree::ColorViz(0.0, 1.0, 0.0, 1.0));

    viz.addModel(modelLoader.model(), "robot");

    yarp::sig::FlexImage image;
    image.setPixelCode(VOCAB_PIXEL_RGB);
    image.resize(textureOptions.winWidth, textureOptions.winHeight);

    // initialise yarp network
    yarp::os::Network yarp;
    if (useNetwork && !yarp.checkNetwork())
    {
        yWarning()<<"No YARP network found. Avoiding to use the network.";
        useNetwork = false;
    }

    yarp::os::BufferedPort<yarp::sig::FlexImage> imagePort;
    if (useNetwork)
    {
        imagePort.open("/visualizerImage");
    }
    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IEncodersTimed *encodersInterface{nullptr};
    if (useNetwork && connectToRobot)
    {
        yarp::os::Property remapperOptions;
        remapperOptions.put("device", "remotecontrolboardremapper");
        yarp::os::Bottle axesNames;
        yarp::os::Bottle & axesList = axesNames.addList();
        for (auto& joint : jointList)
        {
            axesList.addString(joint);
        }
        remapperOptions.put("axesNames",axesNames.get(0));

        yarp::os::Bottle remoteControlBoards;
        yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
        for (auto& cb : controlBoards)
        {
            remoteControlBoardsList.addString("/" + robotPrefix + "/" + cb);
        }
        remapperOptions.put("remoteControlBoards",remoteControlBoards.get(0));
        remapperOptions.put("localPortPrefix", "/visualizer/remoteControlBoard:i");

        if(!robotDevice.open(remapperOptions))
        {
            connectToRobot = false;
        }
        if (!robotDevice.view(encodersInterface) || !encodersInterface)
        {
            connectToRobot = false;
        }
    }

    std::vector<iDynTree::PixelViz> pixels;

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastSent = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastViz = std::chrono::steady_clock::now();

    unsigned int desiredFPS = 30;
    unsigned int maxVizFPS = 65;
    bool mirrorImage = false;

    long minimumMicroSec = std::round(1e6 / (double) desiredFPS);
    long minimumMicroSecViz = std::round(1e6 / (double) maxVizFPS);

    iDynTree::Transform wHb = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize joints(jointList.size());
    iDynTree::VectorDynSize jointsInDeg(jointList.size());

    while(viz.run() && !idyntree_yarp_tools::isClosing)
    {
        now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(now - lastViz).count() < minimumMicroSecViz)
        {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        joints.zero();
        if (connectToRobot)
        {
            encodersInterface->getEncoders(jointsInDeg.data());
        }

        for (size_t i = 0; i < jointsInDeg.size(); ++i)
        {
            joints(i) = iDynTree::deg2rad(jointsInDeg(i));
        }
        viz.modelViz("robot").setPositions(wHb, joints);

        viz.draw();
        lastViz = std::chrono::steady_clock::now();

        if (useNetwork && std::chrono::duration_cast<std::chrono::microseconds>(now - lastSent).count() >= minimumMicroSec)
        {

            if (textureInterface->getPixels(pixels))
            {
                for (unsigned int i = 0; i < pixels.size(); ++i)
                {
                    iDynTree::PixelViz& pixelImage = pixels[i];

                    size_t width;
                    if (mirrorImage)
                    {
                        width = image.width() - 1 - pixelImage.width;
                    }
                    else
                    {
                        width = pixelImage.width;
                    }

                    yarp::sig::PixelRgb& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgb*>(
                                                           image.getPixelAddress(width, pixelImage.height)));

                    pixelYarp.r = pixelImage.r;
                    pixelYarp.g = pixelImage.g;
                    pixelYarp.b = pixelImage.b;
                }
            }
            yarp::sig::FlexImage& imageToBeSent = imagePort.prepare();
            imageToBeSent.setPixelCode(VOCAB_PIXEL_RGB);
            imageToBeSent.setExternal(image.getRawImage(), image.width(), image.height()); //Avoid to copy
            imagePort.write();
            lastSent = std::chrono::steady_clock::now();
        }
    }

    image.resize(0,0);
    imagePort.close();
    robotDevice.close();

    return 0;
}
