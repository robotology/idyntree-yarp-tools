#include "Visualizer.h"

using namespace std::chrono_literals;

bool idyntree_yarp_tools::Visualizer::configure()
{
    // Listen to signals for closing in a clean way the application
    idyntree_yarp_tools::handleSignals([this](){this->closeSignalHandler();});

    std::lock_guard<std::mutex> lock(m_mutex);
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
    modelLoader.loadReducedModelFromFile(pathToModel, jointList);

    viz.init(options);
    textureInterface = viz.textures().add("AdditionalTexture", textureOptions);

    viz.camera().setPosition(iDynTree::Position(1.2, 0.0, 0.5));
    viz.camera().setTarget(iDynTree::Position(-0.15, 0.0, 0.15));
    viz.camera().animator()->enableMouseControl(true);

    double sqrt2 = std::sqrt(2.0);
    viz.enviroment().lightViz("sun").setDirection(iDynTree::Direction(0.5/sqrt2, 0, -0.5/sqrt2));
    viz.enviroment().addLight("secondSun");
    viz.enviroment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    viz.enviroment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));

    textureInterface->environment().lightViz("sun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));
    textureInterface->environment().addLight("secondSun");
    textureInterface->environment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    textureInterface->environment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));
    textureInterface->environment().setElementVisibility("floor_grid", false);
    textureInterface->environment().setElementVisibility("world_frame", false);
    textureInterface->environment().setBackgroundColor(iDynTree::ColorViz(0.0, 0.0, 0.0, 0.0));

    viz.addModel(modelLoader.model(), "robot");

    image.setPixelCode(VOCAB_PIXEL_RGB);
    image.resize(textureOptions.winWidth, textureOptions.winHeight);

    // initialise yarp network
    yarp::os::Network yarp;
    if (useNetwork && !yarp.checkNetwork())
    {
        yWarning()<<"No YARP network found. Avoiding to use the network.";
        useNetwork = false;
        connectToRobot = false;
    }

    if (useNetwork)
    {
        imagePort.open("/visualizerImage");
    }

    if (connectToRobot)
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


    now = std::chrono::steady_clock::now();
    lastSent = std::chrono::steady_clock::now();
    lastViz = std::chrono::steady_clock::now();

    minimumMicroSec = std::round(1e6 / (double) desiredFPS);
    minimumMicroSecViz = std::round(1e6 / (double) maxVizFPS);

    wHb = iDynTree::Transform::Identity();
    joints.resize(jointList.size());
    jointsInDeg.resize(jointList.size());

    return true;
}

int idyntree_yarp_tools::Visualizer::run()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        while(viz.run() && !m_isClosing)
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

                for (size_t i = 0; i < jointsInDeg.size(); ++i)
                {
                    joints(i) = iDynTree::deg2rad(jointsInDeg(i));
                }
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
    }

    close();
    return EXIT_SUCCESS;


}

void idyntree_yarp_tools::Visualizer::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    image.resize(0,0);
    imagePort.close();
    robotDevice.close();
}

void idyntree_yarp_tools::Visualizer::closeSignalHandler()
{
    m_isClosing = true;
}
