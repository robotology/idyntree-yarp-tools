#include "Visualizer.h"

using namespace std::chrono_literals;

bool idyntree_yarp_tools::Visualizer::connectToTheRobot()
{
    m_connectedToTheRobot = false;

    switch (m_connectionType)
    {
    case ConnectionType::REMAPPER:
        if (!m_remapperConnector.connectToRobot())
        {
            return false;
        }
        break;

    case ConnectionType::STATE_EXT:
        if (!m_stateExtConnector.connectToRobot())
        {
            return false;
        }
        break;
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

    yarp::os::Value floorVisibleValue = inputConf.find("floorVisible");

    if (!floorVisibleValue.isNull())
    {
        if (!floorVisibleValue.isBool())
        {
            yError() << "floorVisible is specified but it is not a bool.";
            return false;
        }
        environment.setElementVisibility("floor_grid", floorVisibleValue.asBool());
    }

    yarp::os::Value worldFrameVisibleValue = inputConf.find("worldFrameVisible");

    if (!worldFrameVisibleValue.isNull())
    {
        if (!worldFrameVisibleValue.isBool())
        {
            yError() << "worldFrameVisible is specified but it is not a bool.";
            return false;
        }
        environment.setElementVisibility("world_frame", worldFrameVisibleValue.asBool());
    }

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
                yError() << "The value in position " << i << " (0-based) of backgroundColor is not a double.";
                return false;
            }
        }

        camera.setTarget(desiredTarget);
    }

    camera.animator()->enableMouseControl(true);

    return true;
}

void idyntree_yarp_tools::Visualizer::updateJointValues()
{
    if (m_connectedToTheRobot)
    {
        switch (m_connectionType)
        {
        case ConnectionType::REMAPPER:
            m_remapperConnector.getJointValues(m_joints);
            break;

        case ConnectionType::STATE_EXT:
            m_stateExtConnector.getJointValues(m_joints);
            break;
        }
    }
}

bool idyntree_yarp_tools::Visualizer::configure(const yarp::os::ResourceFinder &rf)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_basicInfo = std::make_shared<BasicInfo>();

    // Listen to signals for closing in a clean way the application
    idyntree_yarp_tools::handleSignals([this](){this->closeSignalHandler();});

    if (!rf.isNull())
    {
        yInfo() << "Configuring with the following options:\n" << rf.toString();
    }

    m_offline = rf.check("offline");

    // initialise yarp network
    yarp::os::Network yarpNetwork;
    if (!m_offline && !yarpNetwork.checkNetwork())
    {
        yError()<<"No YARP network found. Run with --offline if you don't want to use the network.";
        return false;
    }

    std::string modelName = rf.check("model", yarp::os::Value("model.urdf")).asString();

    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(modelName);
    m_modelLoader.loadModelFromFile(pathToModel);

    std::string localName;

    {
        std::lock_guard<std::mutex> lock(m_basicInfo->mutex);
        m_basicInfo->name = rf.check("name", yarp::os::Value("idyntree-yarp-visualizer")).asString();
        localName = m_basicInfo->name;
        m_basicInfo->robotPrefix = rf.check("robot", yarp::os::Value("icub")).asString();
    }

    if (m_stateExtConnector.usingStateExt(rf))
    {
        m_connectionType = ConnectionType::STATE_EXT;

        if (!m_stateExtConnector.configure(rf, m_basicInfo))
        {
            yError() << "Failed to configure the module to connect to the robot via the StateExt port.";
            return false;
        }
    }
    else
    {
        m_connectionType = ConnectionType::REMAPPER;

        if (!m_remapperConnector.configure(rf, m_modelLoader.model(), m_basicInfo))
        {
            yError() << "Failed to configure the module to connect to the robot via RemoteControlBoardRemapper.";
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_basicInfo->mutex);
        std::stringstream jointListInfo;
        jointListInfo << "Using the following joints for visualization:" << std::endl;
        for (const std::string& joint : m_basicInfo->jointList)
        {
            jointListInfo << "    - " << joint <<std::endl;
        }

        yInfo() << jointListInfo.str();

        if (!m_modelLoader.loadReducedModelFromFullModel(m_modelLoader.model(), m_basicInfo->jointList)) //The connectors take care of setting the joint list
        {
            yError() << "Failed to get reduced model.";
            return false;
        }

        m_joints.resize(m_basicInfo->jointList.size());
        m_joints.zero();
    }

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
        std::string rpcPortName = "/" + localName + "/rpc";
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

            std::string streamPortNameFull = "/" + localName + "/" + streamPortName;
            if (!m_imagePort.open(streamPortNameFull))
            {
                yError() << "Failed to open port " << streamPortNameFull;
                return false;
            }

            iDynTree::VisualizerOptions textureOptions;

            //default values
            textureOptions.winHeight = 400;
            textureOptions.winWidth = 400;
            m_desiredFPS = 30;

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

            //Default values
            m_textureInterface->environment().setBackgroundColor(iDynTree::ColorViz(0.0, 0.0, 0.0, 1.0));
            m_textureInterface->environment().setFloorGridColor(iDynTree::ColorViz(0.0, 0.0, 1.0, 1.0));
            m_textureInterface->environment().setElementVisibility("floor_grid", false);
            m_textureInterface->environment().setElementVisibility("world_frame", false);

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

    return true;
}

bool idyntree_yarp_tools::Visualizer::neededHelp(const yarp::os::ResourceFinder &rf)
{

    if (rf.check("help"))
    {
        std::cout << "Usage: idyntree-yarp-visualizer" << std::endl
                  << "Optional arguments:" << std::endl
                  << "--name <name>                                      The prefix used to open the visualizer ports. Default: idyntree-yarp-visualizer;" << std::endl << std::endl
                  << "--robot <robot>                                    The prefix used to connect to the robot. Default: icub;" << std::endl << std::endl
                  << "--model <model>                                    The URDF model to open. Default: model.urdf, it will be found according to the YARP_ROBOT_NAME;" << std::endl << std::endl
                  << "--offline                                          Avoid to use the network. The model is only visualized;" << std::endl << std::endl
                  << "--autoconnect [true|false]                         If set to true, or no value is provided, it will try to connect to the robot automatically at startup." << std::endl
                  << "                                                   If fails, the visualizer does not start. By default it tries to connect to the robot, but if it fails, nothing happens;" << std::endl << std::endl
                  << "--controlboards \"(<cb1>, ...)\"                     The set of control boards to connect to. Default: \"(head, torso, left_arm, right_arm, left_leg, right_leg)\";" << std::endl << std::endl
                  << "--joints \"(<j1>, ...)\"                             The set of joints to consider. These names are used both in the model and when connecting to the robot." << std::endl
                  << "                                                   By default, it uses all the joints specified in the model having one degree of freedom;" << std::endl << std::endl
                  << "--connectToStateExt \"(<cb1>, (<j1>, ...), ..)\"     With this command, it is possible to connect to the stateExt ports opened by the robot. This allows to visualize also data" << std::endl
                  << "                                                   saved through the yarpdatatdumper for example. The stateExt ports contain only joint values, without any other information," << std::endl
                  << "                                                   hence, it is necessary to explicitly list all the joints of the selected control boards in their correct order. " << std::endl
                  << "                                                   The list provided after connectToStateExt is supposed to have an even number of elements." << std::endl
                  << "                                                   It needs to be a sequence of a list containing the name of the control board (and eventually the number of joints to consider)," << std::endl
                  << "                                                   followed by the full list of joints in the control board. Each joint entry can also be a list. In this case," << std::endl
                  << "                                                   the first element is the joint name, the second is a keyword for the " << std::endl
                  << "                                                   joint type (\"p\" or \"prismatic\" for prismatic joints, \"r\" or \"revolute\" for revolute joints)." << std::endl
                  << "                                                   If a joint name is specified without being a list, then it is assumed to be a revolute joint."  << std::endl
                  << "                                                   For example, suppose you want to connect to the neck and the torso, the syntax would be:"  << std::endl
                  << "                                                   --connectToStateExt \"(head, (neck_pitch, neck_roll, neck_yaw), torso, (torso_pitch, torso_roll, torso_yaw)\"" << std::endl
                  << "                                                   If, for example, neck_roll is a prismatic joint, the syntax would be:" << std::endl
                  << "                                                   --connectToStateExt \"(head, (neck_pitch, (neck_roll, p), neck_yaw), torso, (torso_pitch, torso_roll, torso_yaw)\"" << std::endl
                  << "                                                   In some case, it might be useful to consider only the first n joints of a control board (for example to avoid considering the hand joints)." << std::endl
                  << "                                                   It is still necessary to specify all the joints, but only the first n will be used for the visualization." << std::endl
                  << "                                                   For example, if you want to avoid considering the torso_yaw in the visualization, use the following notation:" << std::endl
                  << "                                                   --connectToStateExt \"(head, (neck_pitch, (neck_roll, p), neck_yaw), (torso, 2), (torso_pitch, torso_roll, torso_yaw)\"" << std::endl
                  << "                                                   In case --robot is \"icub\" or \"icubSim\" it is possible to use the following simplified syntax to connect to all the supported joints:" << std::endl
                  << "                                                   --connectToStateExt default" << std::endl
                  << "                                                   When using connectToStateExt, the --controlboards and --joints options are ignored." << std::endl << std::endl
                  << "--cameraPosition \"(px, py, pz)\"                    Camera initial position. Default \"(0.8, 0.8, 0.8)\";" << std::endl << std::endl
                  << "--imageWidth <width>                               The initial width of the visualizer window. Default 800;" << std::endl << std::endl
                  << "--imageHeight <height>                             The initial height of the visualizer window. Default 600;" << std::endl << std::endl
                  << "--maxFPS <fps>                                     The maximum frame per seconds to update the visualizer. Default 65;" << std::endl << std::endl
                  << "--backgroundColor \"(r, g, b)\"                      Visualizer background color. Default \"(0.0, 0.4, 0.4)\";" << std::endl << std::endl
                  << "--floorGridColor \"(r, g, b)\"                       Visualizer floor grid color. Default \"(0.0, 0.0, 1.0)\";" << std::endl << std::endl
                  << "--floorVisible <true|false>                        Set the visibility of the visualizer floor grid. Default true;" << std::endl << std::endl
                  << "--worldFrameVisible <true|false>                   Set the visibility of the visualizer world frame. Default true;" << std::endl << std::endl
                  << "--streamImage <true|false>                         If set to true, the visualizer can publish on a port what is rendered in the visualizer. Default true;" << std::endl << std::endl << std::endl
                  << " The following optional arguments are used only if the streaming of the image is enabled (see --streamImage):" << std::endl << std::endl
                  << "--OUTPUT_STREAM::portName <name>                   The suffix of the port where the stream the image. Default \"image\";" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::imageWidth <width>                The width of the image streamed on the port. Default 400;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::imageHeight <height>              The height of the image streamed on the port. Default 400;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::maxFPS <fps>                      The maximum number of times per second the image is streamed on the network. Default 30;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::mirrorImage <true|false>          If true, it mirrors the image before streaming it. Default false;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::floorVisible <true|false>         Set the visibility of the floor grid in the streamed image. Default false;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::worldFrameVisible <true|false>    Set the visibility of the world frame in the streamed image. Default false;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::backgroundColor \"(r, g, b)\"       Set the background color of the streamed image. Default \"(0.0, 0.0, 0.0)\";" << std::endl << std::endl
                  << "--OUTPUT_STREAM::floorGridColor \"(r, g, b)\"        Set the floor grid color of the streamed image. Default \"(0.0, 0.0, 1.0)\";" << std::endl << std::endl << std::endl
                  << "All these options can be added to a .ini file. If you use the following argument:" << std::endl
                  << "--from </path/file>.ini                            Example of .ini file:" << std::endl
                  << "                                                   #--------------------"<< std::endl
                  << "                                                   name    anotherName" << std::endl
                  << "                                                   robot   myRobot" << std::endl
                  << "                                                   [OUTPUT_STREAM]" <<std::endl
                  << "                                                   portName    myPort" << std::endl
                  << "                                                   mirrorImage true" << std::endl
                  << "                                                   floorGridColor (1.0, 0.0, 0.0)" << std::endl
                  << "                                                   #--------------------" <<std::endl
                  << "                                                   Note that all the -- have been removed, while the prefix OUTPUT_STREAM::" <<std::endl
                  << "                                                   is no more necessary after the [OUTPUT_STREAM] tag." <<std::endl << std::endl;

        return true;
    }
    else
    {
        return false;
    }
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

    updateJointValues();

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

    m_viz.close();
    m_imagePort.close();
    m_rpcPort.close();
    m_remapperConnector.close();
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

std::vector<double> idyntree_yarp_tools::Visualizer::getCameraPosition()
{
    std::vector<double> output(3);
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        iDynTree::Position pos =  m_viz.camera().getPosition();
        output[0] = pos[0];
        output[1] = pos[1];
        output[2] = pos[2];

        return output;
    }
}

std::vector<double> idyntree_yarp_tools::Visualizer::getCameraTarget()
{
    std::vector<double> output(3);
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        iDynTree::Position target =  m_viz.camera().getTarget();
        output[0] = target[0];
        output[1] = target[1];
        output[2] = target[2];

        return output;
    }
}
