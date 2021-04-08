#include "Visualizer.h"

using namespace std::chrono_literals;

bool idyntree_yarp_tools::Visualizer::connectToTheRobot()
{
    m_connectedToTheRobot = false;
    m_robotDevice.close();
    m_encodersInterface = nullptr;

    yarp::os::Property remapperOptions;
    remapperOptions.put("device", "remotecontrolboardremapper");
    yarp::os::Bottle axesNames;
    yarp::os::Bottle & axesList = axesNames.addList();
    for (auto& joint : m_jointList)
    {
        axesList.addString(joint);
    }
    remapperOptions.put("axesNames",axesNames.get(0));

    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
    for (auto& cb : m_controlBoards)
    {
        remoteControlBoardsList.addString("/" + m_robotPrefix + "/" + cb);
    }
    remapperOptions.put("remoteControlBoards",remoteControlBoards.get(0));
    remapperOptions.put("localPortPrefix", "/" + m_name +"/remoteControlBoard:i");

    if(!m_robotDevice.open(remapperOptions))
    {
        yError() << "Failed to open remote control board remapper.";
        return false;
    }
    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "Failed to view encoder interface.";

        return false;
    }

    m_connectedToTheRobot = true;

    return true;
}

bool idyntree_yarp_tools::Visualizer::setVizOptionsFromConfig(const yarp::os::Searchable &inputConf, iDynTree::VisualizerOptions& output, unsigned int &fps)
{
    yarp::os::Value width = inputConf.find("imageWidth");

    if (!width.isNull())
    {
        if (width.isInt() && width.asInt() >= 0)
        {
            output.winWidth = width.asInt();
        }
        else
        {
            yError() << "imageWidth is specified, but it is not a positive integer.";
            return false;
        }
    }


    yarp::os::Value height = inputConf.find("imageHeight");

    if (!height.isNull())
    {
        if (height.isInt() && height.asInt() >= 0)
        {
            output.winHeight = height.asInt();
        }
        else
        {
            yError() << "imageHeight is specified, but it is not a positive integer.";
            return false;
        }
    }

    yarp::os::Value fpsValue = inputConf.find("maxFPS");

    if (!fpsValue.isNull())
    {
        if (fpsValue.isInt() && fpsValue.asInt() >= 0)
        {
            fps = fpsValue.asInt();
        }
        else
        {
            yError() << "maxFPS is specified, but it is not a positive integer.";
            return false;
        }
    }

    return true;
}

bool idyntree_yarp_tools::Visualizer::setVizEnvironmentFromConfig(const yarp::os::Searchable &inputConf, iDynTree::IEnvironment &environment)
{
    yarp::os::Value backgroundColorValue = inputConf.find("backgroundColor");

    if (!backgroundColorValue.isNull())
    {
        if (!backgroundColorValue.isList())
        {
            yError() << "backgroundColor is specified but it is not a list.";
            return false;
        }

        yarp::os::Bottle* backGroundColorList = backgroundColorValue.asList();
        if (backGroundColorList->size() != 3)
        {
            yError() << "backgroundColor is specified but the size is different from 3 (only RGB colors are supported).";
            return false;
        }

        iDynTree::Vector3 rgb;

        for (size_t i = 0; i< 3; ++i)
        {
            if (backGroundColorList->get(i).isDouble())
            {
                rgb[i] = backGroundColorList->get(i).asDouble();
                if (rgb[i] > 1.0 || rgb[i] < 0.0)
                {
                    yError() << "The value in position " << i << " (0-based) of backgroundColor is not value. It needs to be between 0.0 and 1.0.";
                    return false;
                }
            }
            else
            {
                yError() << "The value in position " << i << " (0-based) of backgroundColor is not a double";
                return false;
            }
        }

        environment.setBackgroundColor(iDynTree::ColorViz(rgb[0], rgb[1], rgb[2], 1.0));
    }

    yarp::os::Value floorgridColorValue = inputConf.find("floorGridColor");

    if (!floorgridColorValue.isNull())
    {
        if (!floorgridColorValue.isList())
        {
            yError() << "floorGridColor is specified but it is not a list.";
            return false;
        }

        yarp::os::Bottle* floorGridColorList = floorgridColorValue.asList();
        if (floorGridColorList->size() != 3)
        {
            yError() << "floorGridColor is specified but the size is different from 3 (only RGB colors are supported).";
            return false;
        }

        iDynTree::Vector3 rgb;

        for (size_t i = 0; i< 3; ++i)
        {
            if (floorGridColorList->get(i).isDouble())
            {
                rgb[i] = floorGridColorList->get(i).asDouble();
                if (rgb[i] > 1.0 || rgb[i] < 0.0)
                {
                    yError() << "The value in position " << i << " (0-based) of floorGridColor is not value. It needs to be between 0.0 and 1.0.";
                    return false;
                }
            }
            else
            {
                yError() << "The value in position " << i << " (0-based) of floorGridColor is not a double";
                return false;
            }
        }

        environment.setFloorGridColor(iDynTree::ColorViz(rgb[0], rgb[1], rgb[2], 1.0));
    }

    environment.setElementVisibility("floor_grid", inputConf.check("floorVisible", yarp::os::Value(true)).asBool());
    environment.setElementVisibility("world_frame", inputConf.check("worldFrameVisible", yarp::os::Value(true)).asBool());

    return true;
}

bool idyntree_yarp_tools::Visualizer::setVizCameraFromConfig(const yarp::os::Searchable &inputConf, iDynTree::ICamera &camera)
{
    yarp::os::Value cameraPosition = inputConf.find("cameraPosition");

    if (!cameraPosition.isNull())
    {
        if (!cameraPosition.isList())
        {
            yError() << "cameraPosition is specified but it is not a list.";
            return false;
        }

        yarp::os::Bottle* cameraPositionList = cameraPosition.asList();
        if (cameraPositionList->size() != 3)
        {
            yError() << "cameraPosition is specified but the size is different from 3.";
            return false;
        }

        iDynTree::Position desiredPosition;

        for (size_t i = 0; i< 3; ++i)
        {
            if (cameraPositionList->get(i).isDouble())
            {
                desiredPosition[i] = cameraPositionList->get(i).asDouble();
            }
            else
            {
                yError() << "The value in position " << i << " (0-based) of backgroundColor is not a double";
                return false;
            }
        }

        camera.setPosition(desiredPosition);
    }

    yarp::os::Value cameraTarget = inputConf.find("cameraTarget");

    if (!cameraTarget.isNull())
    {
        if (!cameraTarget.isList())
        {
            yError() << "cameraTarget is specified but it is not a list.";
            return false;
        }

        yarp::os::Bottle* cameraTargetList = cameraTarget.asList();
        if (cameraTargetList->size() != 3)
        {
            yError() << "cameraTarget is specified but the size is different from 3.";
            return false;
        }

        iDynTree::Position desiredTarget;

        for (size_t i = 0; i< 3; ++i)
        {
            if (cameraTargetList->get(i).isDouble())
            {
                desiredTarget[i] = cameraTargetList->get(i).asDouble();
            }
            else
            {
                yError() << "The value in position " << i << " (0-based) of backgroundColor is not a double";
                return false;
            }
        }

        camera.setTarget(desiredTarget);
    }

    camera.animator()->enableMouseControl(true);

    return true;
}

bool idyntree_yarp_tools::Visualizer::configure(const yarp::os::ResourceFinder &rf)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    // Listen to signals for closing in a clean way the application
    idyntree_yarp_tools::handleSignals([this](){this->closeSignalHandler();});

    m_offline = rf.check("offline");

    // initialise yarp network
    yarp::os::Network yarpNetwork;
    if (!m_offline && !yarpNetwork.checkNetwork())
    {
        yError()<<"No YARP network found. Run with --offline if you don't want to use the network.";
        return false;
    }

    m_name = rf.check("name", yarp::os::Value("idyntree-visualizer")).asString();
    m_robotPrefix = rf.check("robot", yarp::os::Value("icub")).asString();

    std::string modelName = rf.check("model", yarp::os::Value("model.urdf")).asString();

    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(modelName);
    m_modelLoader.loadReducedModelFromFile(pathToModel, m_jointList);

    iDynTree::VisualizerOptions vizOptions;
    m_maxVizFPS = 65;  //default value
    if (!setVizOptionsFromConfig(rf, vizOptions, m_maxVizFPS))
    {
        yError() << "Failed to get the visualizer options from config file.";
        return false;
    }

    if (!m_viz.init(vizOptions))
    {
        yError() << "Failed to initialize the visualizer.";
        return false;
    }

    if (!setVizCameraFromConfig(rf, m_viz.camera()))
    {
        yError() << "Failed to initialize the visualizer camera.";
        return false;
    }

    double sqrt2 = std::sqrt(2.0);
    m_viz.enviroment().lightViz("sun").setDirection(iDynTree::Direction(0.5/sqrt2, 0, -0.5/sqrt2));
    m_viz.enviroment().addLight("secondSun");
    m_viz.enviroment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    m_viz.enviroment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));

    if (!setVizEnvironmentFromConfig(rf, m_viz.enviroment()))
    {
        yError() << "Failed to set visualizer environment.";
        return false;
    }

    m_viz.addModel(m_modelLoader.model(), "robot");

    if (!m_offline)
    {
        std::string rpcPortName = "/" + m_name + "/rpc";
        this->yarp().attachAsServer(this->m_rpcPort);
        if(!m_rpcPort.open(rpcPortName))
        {
            yError() << "Could not open" << rpcPortName << " RPC port.";
            return false;
        }
    }

    bool streamImage = rf.check("streamImage", yarp::os::Value(true)).asBool();

    if (streamImage)
    {
        if (m_offline)
        {
            yWarning() << "streamImage is set to true, but the visualizer is running offline. No port will be opened.";
        }
        else
        {
            yarp::os::Bottle streamGroup = rf.findGroup("OUTPUT_STREAM");

            std::string streamPortName = streamGroup.check("portName", yarp::os::Value("image")).asString();

            std::string streamPortNameFull = "/" + m_name + "/" + streamPortName;
            if (!m_imagePort.open(streamPortNameFull))
            {
                yError() << "Failed to open port " << streamPortNameFull;
                return false;
            }

            iDynTree::VisualizerOptions textureOptions;

            m_desiredFPS = 30; //default value
            if (!setVizOptionsFromConfig(streamGroup, textureOptions, m_desiredFPS))
            {
                yError() << "Failed to set the options of the additional texture.";
                return false;
            }

            m_textureInterface = m_viz.textures().add("AdditionalTexture", textureOptions);

            m_textureInterface->environment().lightViz("sun").setDirection(iDynTree::Direction(0.5/sqrt2, 0, -0.5/sqrt2));
            m_textureInterface->environment().addLight("secondSun");
            m_textureInterface->environment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
            m_textureInterface->environment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));

            if(!setVizEnvironmentFromConfig(streamGroup, m_textureInterface->environment()))
            {
                yError() << "Failed to set the additional texture environment.";
                return false;
            }

            m_mirrorImage = streamGroup.check("mirrorImage", yarp::os::Value("false")).asBool();

            m_image.setPixelCode(VOCAB_PIXEL_RGB);
            m_image.resize(textureOptions.winWidth, textureOptions.winHeight);

        }
    }

    bool autoconnectSpecified = rf.check("autoconnect"); //Check if autoconnect has been explicitly set
    bool autoconnectSpecifiedValue = true;
    if (rf.check("autoconnect")) //autoconnect is present
    {
        yarp::os::Value autoconnectValue = rf.find("autoconnect");

        if (!autoconnectValue.isNull() && autoconnectValue.isBool() && !autoconnectValue.asBool()) //autoconnect is specified to be false
        {
            autoconnectSpecifiedValue = false;
        }

    }
    if (autoconnectSpecifiedValue)
    {
        if (!m_offline)
        {
            bool ok = connectToTheRobot();
            if (autoconnectSpecified && !ok)
            {
                yError() << "It is specified to autoconnect, but the connection to the robot failed.";
                return false;
            }
        }
        else
        {
            if (autoconnectSpecified)
            {
                yError() << "It is specified to autoconnect, but it is also specified to run offline.";
                return false;
            }
        }
    }

    m_now = std::chrono::steady_clock::now();
    m_lastSent = std::chrono::steady_clock::now();
    m_lastViz = std::chrono::steady_clock::now();

    m_minimumMicroSec = std::round(1e6 / (double) m_desiredFPS);
    m_minimumMicroSecViz = std::round(1e6 / (double) m_maxVizFPS);

    m_wHb = iDynTree::Transform::Identity();
    m_joints.resize(m_jointList.size());
    m_jointsInDeg.resize(m_jointList.size());
    m_joints.zero();

    return true;
}

int idyntree_yarp_tools::Visualizer::run()
{
    while(!m_isClosing)
    {
        m_now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(m_now - m_lastViz).count() < m_minimumMicroSecViz)
        {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        if (!update())
        {
            m_isClosing = true;
        }

    }

    close();
    return EXIT_SUCCESS;
}

bool idyntree_yarp_tools::Visualizer::update()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_viz.run())
    {
        m_isClosing = true;
        return true;
    }

    if (m_connectedToTheRobot)
    {
        m_encodersInterface->getEncoders(m_jointsInDeg.data());

        for (size_t i = 0; i < m_jointsInDeg.size(); ++i)
        {
            m_joints(i) = iDynTree::deg2rad(m_jointsInDeg(i));
        }
    }

    m_viz.modelViz("robot").setPositions(m_wHb, m_joints);

    m_viz.draw();
    m_lastViz = std::chrono::steady_clock::now();

    if (!m_offline && m_textureInterface && std::chrono::duration_cast<std::chrono::microseconds>(m_now - m_lastSent).count() >= m_minimumMicroSec)
    {

        if (m_textureInterface->getPixels(m_pixels))
        {
            for (unsigned int i = 0; i < m_pixels.size(); ++i)
            {
                iDynTree::PixelViz& pixelImage = m_pixels[i];

                size_t width;
                if (m_mirrorImage)
                {
                    width = m_image.width() - 1 - pixelImage.width;
                }
                else
                {
                    width = pixelImage.width;
                }

                yarp::sig::PixelRgb& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgb*>(
                                                       m_image.getPixelAddress(width, pixelImage.height)));

                pixelYarp.r = pixelImage.r;
                pixelYarp.g = pixelImage.g;
                pixelYarp.b = pixelImage.b;
            }
        }
        yarp::sig::FlexImage& imageToBeSent = m_imagePort.prepare();
        imageToBeSent.setPixelCode(VOCAB_PIXEL_RGB);
        imageToBeSent.setExternal(m_image.getRawImage(), m_image.width(), m_image.height()); //Avoid to copy
        m_imagePort.write();
        m_lastSent = std::chrono::steady_clock::now();
    }

    return true;
}

void idyntree_yarp_tools::Visualizer::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_image.resize(0,0);
    m_imagePort.close();
    m_rpcPort.close();
    m_robotDevice.close();
}

void idyntree_yarp_tools::Visualizer::closeSignalHandler()
{
    m_isClosing = true;
}

bool idyntree_yarp_tools::Visualizer::setBasePosition(const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_wHb.setPosition(iDynTree::Position(x, y, z));
    return true;
}

bool idyntree_yarp_tools::Visualizer::setBaseRotation(const double roll, const double pitch, const double yaw)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    double scaling = M_PI/180;
    m_wHb.setRotation(iDynTree::Rotation::RPY(scaling * roll, scaling * pitch, scaling * yaw));
    return true;
}

bool idyntree_yarp_tools::Visualizer::setBasePose(const double x, const double y, const double z, const double roll, const double pitch, const double yaw)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_wHb.setPosition(iDynTree::Position(x, y, z));
    double scaling = M_PI/180;
    m_wHb.setRotation(iDynTree::Rotation::RPY(scaling * roll, scaling * pitch, scaling * yaw));
    return true;
}

bool idyntree_yarp_tools::Visualizer::reconnectToRobot()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return connectToTheRobot();
}
