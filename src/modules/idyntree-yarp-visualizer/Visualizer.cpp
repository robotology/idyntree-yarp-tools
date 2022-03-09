#include "Visualizer.h"

using namespace std::chrono_literals;

void idyntree_yarp_tools::Visualizer::connectToTheRobot()
{
    m_connectedToTheRobot = false;
    m_connectedToWBD = false;

    switch (m_connectionType)
    {
    case ConnectionType::REMAPPER:
        if (m_remapperConnector.connectToRobot())
        {
            m_connectedToTheRobot = true;
        }
        break;

    case ConnectionType::STATE_EXT:
        if (m_stateExtConnector.connectToRobot())
        {
            m_connectedToTheRobot = true;
        }
        break;
    }

    if (m_useWBD)
    {
        m_connectedToWBD = yarp::os::Network::connect(m_remoteNextExternalWrenchesPortName, m_netExternalWrenchesPort.getName(), "fast_tcp");
    }

}

iDynTree::Vector4 idyntree_yarp_tools::Visualizer::rgbaFromConfig(const yarp::os::Searchable &inputConf, const std::string &optionName)
{
    iDynTree::Vector4 rgba;
    rgba.zero();
    rgba[3] = 1.0;
    rgba[1] = -1; //g=-1 -> The option has not been found
    yarp::os::Value colorValue = inputConf.find(optionName);

    if (!colorValue.isNull())
    {
        if (!colorValue.isList())
        {
            yError() << optionName + " is specified but it is not a list.";
            rgba[0] = -1; //r=-1 -> Error
            return rgba;
        }

        yarp::os::Bottle* colorList = colorValue.asList();
        if ((colorList->size() != 3) && (colorList->size() != 4))
        {
            yError() << optionName + " is specified but the size is supposed to be either 3 (RGB) or 4 (RGBA).";
            rgba[0] = -1; //r=-1 -> Error
            return rgba;
        }

        for (size_t i = 0; i< colorList->size(); ++i)
        {
            if (colorList->get(i).isFloat64())
            {
                rgba[i] = colorList->get(i).asFloat64();
                if (rgba[i] > 1.0 || rgba[i] < 0.0)
                {
                    yError() << "The value in position" << i << "(0-based) of " + optionName + " is not an expected value. It needs to be between 0.0 and 1.0.";
                    rgba[0] = -1; //r=-1 -> Error
                    return rgba;
                }
            }
            else
            {
                yError() << "The value in position" << i << "(0-based) of " + optionName + " is not a double";
                rgba[0] = -1; //r=-1 -> Error

                return rgba;
            }
        }
    }
    return rgba;
}

bool idyntree_yarp_tools::Visualizer::setVizOptionsFromConfig(const yarp::os::Searchable &inputConf, iDynTree::VisualizerOptions& output, unsigned int &fps)
{
    yarp::os::Value width = inputConf.find("imageWidth");

    if (!width.isNull())
    {
        if (width.isInt32() && width.asInt32() >= 0)
        {
            output.winWidth = width.asInt32();
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
        if (height.isInt32() && height.asInt32() >= 0)
        {
            output.winHeight = height.asInt32();
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
        if (fpsValue.isInt32() && fpsValue.asInt32() >= 0)
        {
            fps = fpsValue.asInt32();
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
    iDynTree::Vector4 rgbaBackground = rgbaFromConfig(inputConf, "backgroundColor");
    if (rgbaBackground[0] < 0)
    {
        return false; //Error case
    }
    else if (rgbaBackground[1] >= 0) //The option has been found
    {
        environment.setBackgroundColor(iDynTree::ColorViz(rgbaBackground[0], rgbaBackground[1], rgbaBackground[2], rgbaBackground[3]));
    }

    iDynTree::Vector4 rgbaFloor = rgbaFromConfig(inputConf, "floorGridColor");
    if (rgbaFloor[0] < 0)
    {
        return false; //Error case
    }
    else if (rgbaFloor[1] >= 0) //The option has been found
    {
        environment.setFloorGridColor(iDynTree::ColorViz(rgbaFloor[0], rgbaFloor[1], rgbaFloor[2], rgbaFloor[3]));
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
            if (cameraPositionList->get(i).isFloat64())
            {
                desiredPosition[i] = cameraPositionList->get(i).asFloat64();
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
            if (cameraTargetList->get(i).isFloat64())
            {
                desiredTarget[i] = cameraTargetList->get(i).asFloat64();
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

void idyntree_yarp_tools::Visualizer::updateWrenchesVisualization()
{

    size_t inactivityThreshold = m_maxVizFPS/2;

    for (auto& vizWrench : m_netExternalWrenchesMap)
    {
        if (vizWrench.second.inactivityCounter < inactivityThreshold)
        {
            vizWrench.second.inactivityCounter++;
        }
    }

    if (m_connectedToWBD)
    {
        yarp::os::Bottle* bottle = m_netExternalWrenchesPort.read(false);
        if (bottle)
        {
            for (size_t link = 0; link < bottle->size(); ++link)
            {
                yarp::os::Bottle* wrenchPair = bottle->get(link).asList();
                if (!wrenchPair || wrenchPair->size() != 2)
                {
                    return;
                }

                std::string linkName = wrenchPair->get(0).asString();
                yarp::os::Bottle* wrenchBottle = wrenchPair->get(1).asList();

                if (!wrenchBottle || wrenchBottle->size() != 6)
                {
                    return;
                }

                VisualizedWrench& vizWrench = m_netExternalWrenchesMap[linkName];

                vizWrench.inactivityCounter = 0;

                for (size_t i = 0; i < 3; ++i)
                {
                    vizWrench.scaledWrench(i) = m_forceMultiplier * wrenchBottle->get(i).asFloat64();
                }

                for (size_t i = 3; i < 6; ++i)
                {
                    vizWrench.scaledWrench(i) = m_torquesMultiplier * wrenchBottle->get(i).asFloat64();
                }

                if ((vizWrench.frameIndex == iDynTree::FRAME_INVALID_INDEX) && !vizWrench.skip) //the link has never been checked in the model
                {
                    vizWrench.frameIndex = m_viz.modelViz("robot").model().getFrameIndex(linkName);
                    vizWrench.skip =  vizWrench.frameIndex == iDynTree::FRAME_INVALID_INDEX;
                }
            }
        }
    }
    else
    {
        for (auto& vizWrench : m_netExternalWrenchesMap)
        {
            vizWrench.second.scaledWrench.zero();
        }
    }

    for (auto& vizWrench : m_netExternalWrenchesMap)
    {
        if (vizWrench.second.inactivityCounter >= inactivityThreshold)
        {
            vizWrench.second.scaledWrench.zero();
        }

        if (!vizWrench.second.skip)
        {
            iDynTree::Transform linkTransform = m_viz.modelViz("robot").getWorldFrameTransform(vizWrench.second.frameIndex);
            iDynTree::Wrench scaledWrenchInWorld = linkTransform * vizWrench.second.scaledWrench; //the input wrenches are defined in the link frame
            iDynTree::Position originLinear = linkTransform.getPosition();
            iDynTree::Position originAngular = linkTransform.getPosition();


            for (size_t i = 0; i < 3; ++i)
            {
                originLinear(i) -= scaledWrenchInWorld.getLinearVec3()(i); //to have the arrow pointing toward the link
                originAngular(i) -= scaledWrenchInWorld.getAngularVec3()(i);
            }

            if (vizWrench.second.arrowIndexLinear < 0) //The arrow was never added
            {
                vizWrench.second.arrowIndexLinear = m_viz.vectors().addVector(originLinear, scaledWrenchInWorld.getLinearVec3());
                m_viz.vectors().setVectorColor(vizWrench.second.arrowIndexLinear, m_forcesColor);
                vizWrench.second.arrowIndexAngular = m_viz.vectors().addVector(originAngular, scaledWrenchInWorld.getAngularVec3());
                m_viz.vectors().setVectorColor(vizWrench.second.arrowIndexAngular, m_torquesColor);

            }
            else
            {
                m_viz.vectors().updateVector(vizWrench.second.arrowIndexLinear, originLinear, scaledWrenchInWorld.getLinearVec3());
                m_viz.vectors().updateVector(vizWrench.second.arrowIndexAngular, originAngular, scaledWrenchInWorld.getAngularVec3());
            }
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

    if (pathToModel == "")
    {
        yError() << "Failed to find" << modelName;
        return false;
    }

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
    m_viz.environment().lightViz("sun").setDirection(iDynTree::Direction(0.5/sqrt2, 0, -0.5/sqrt2));
    m_viz.environment().addLight("secondSun");
    m_viz.environment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    m_viz.environment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));

    if (!setVizEnvironmentFromConfig(rf, m_viz.environment()))
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
            m_desiredTextureFPS = 30;

            if (!setVizOptionsFromConfig(streamGroup, textureOptions, m_desiredTextureFPS))
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
            m_textureInterface->environment().setBackgroundColor(iDynTree::ColorViz(0.0, 0.0, 0.0, 0.0));
            m_textureInterface->environment().setFloorGridColor(iDynTree::ColorViz(0.0, 0.0, 1.0, 1.0));
            m_textureInterface->environment().setElementVisibility("floor_grid", false);
            m_textureInterface->environment().setElementVisibility("world_frame", false);

            if(!setVizEnvironmentFromConfig(streamGroup, m_textureInterface->environment()))
            {
                yError() << "Failed to set the additional texture environment.";
                return false;
            }

            m_useRGBA = streamGroup.check("useRGBA", yarp::os::Value("false")).asBool();

            m_mirrorImage = streamGroup.check("mirrorImage", yarp::os::Value("false")).asBool();

            m_image.setPixelCode(m_useRGBA ? VOCAB_PIXEL_RGBA : VOCAB_PIXEL_RGB);
            m_image.resize(textureOptions.winWidth, textureOptions.winHeight);

        }
    }

    m_useWBD = ! m_offline && !rf.check("noNetExternalWrenches");
    m_remoteNextExternalWrenchesPortName = rf.check("netExternalWrenchesPortName" , yarp::os::Value("/wholeBodyDynamics/netExternalWrenches:o")).asString();
    m_forceMultiplier = rf.check("externalForcesMultiplier", yarp::os::Value(0.005)).asFloat64();
    m_torquesMultiplier = rf.check("externalTorquesMultiplier", yarp::os::Value(0.05)).asFloat64();
    iDynTree::Vector4 rgbaForces = rgbaFromConfig(rf, "externalForcesColor");
    if (rgbaForces[0] < 0)
    {
        return false; //Error case
    }
    else if (rgbaForces[1] >= 0) //The option has been found
    {
        m_forcesColor.r = rgbaForces(0);
        m_forcesColor.g = rgbaForces(1);
        m_forcesColor.b = rgbaForces(2);
        m_forcesColor.a = rgbaForces(3);
    }
    else
    {
        m_forcesColor = iDynTree::ColorViz(1.0, 0.0, 0.0, 1.0);
    }

    iDynTree::Vector4 rgbaTorques = rgbaFromConfig(rf, "externalTorquesColor");
    if (rgbaTorques[0] < 0)
    {
        return false; //Error case
    }
    else if (rgbaTorques[1] >= 0) //The option has been found
    {
        m_torquesColor.r = rgbaTorques(0);
        m_torquesColor.g = rgbaTorques(1);
        m_torquesColor.b = rgbaTorques(2);
        m_torquesColor.a = rgbaTorques(3);
    }
    else
    {
        m_torquesColor = iDynTree::ColorViz(0.0, 0.0, 1.0, 1.0);
    }


    if (!m_offline && m_useWBD)
    {
        if (!m_netExternalWrenchesPort.open("/" + localName + "/netExternalWrenches:i"))
        {
            yError() << "Failed to open the port /" + localName +"/netExternalWrenches:i to connect to the next external wrenches."
                     << "Use --noNetExternalWrenches to avoid opening it.";
            return false;
        }
    }

    bool autoconnectSpecified = rf.check("autoconnect"); //Check if autoconnect has been explicitly set
    bool autoconnectSpecifiedValue = true;
    if (autoconnectSpecified) //autoconnect is present
    {
        yarp::os::Value& autoconnectValue = rf.find("autoconnect");

        if (!autoconnectValue.isNull() && autoconnectValue.isBool() && !autoconnectValue.asBool()) //autoconnect is specified to be false
        {
            autoconnectSpecifiedValue = false;
        }

    }
    if (autoconnectSpecifiedValue)
    {
        if (!m_offline)
        {
            connectToTheRobot();
            if (autoconnectSpecified && !m_connectedToTheRobot)
            {
                yError() << "It is specified to autoconnect, but the connection to the robot failed.";
                return false;
            }

            if (autoconnectSpecified && !m_connectedToWBD)
            {
                yError() << "It is specified to autoconnect, but the connection to the net external wrenches port failed."
                         << "Use --noNetExternalWrenches to avoid connecting to it.";
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

    m_minimumMicroSec = std::round(1e6 / (double) m_desiredTextureFPS);
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
                  << "--help                                             Print this message;" << std::endl << std::endl
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
                  << "                                                   When using connectToStateExt, the --controlboards and --joints options are ignored;" << std::endl << std::endl
                  << "--noNetExternalWrenches                            Avoid connecting to WholeBodyDynamics to retrieve the net external wrenches applied on the robot link;" << std::endl << std::endl
                  << "--netExternalWrenchesPortName <name>               The name of the WholeBodyDynamics port to retrieve the net external wrenches." << std::endl
                  << "                                                   The port is supposed to send a bottle of n pairs, where n is the number of links." << std::endl
                  << "                                                   The first element of each pair is the name of the link. The second element is yet another list of 6 elements containing the value of the" << std::endl
                  << "                                                   net external wrench, defined in the link frame. The first three elements are the linear part, while the other three are the angular part." << std::endl
                  << "                                                   Default /wholeBodyDynamics/netExternalWrenches:o;"  << std::endl << std::endl
                  << "--externalForcesMultiplier <multiplier>            The multiplier to scale the visualization of the external forces. Default 0.005"  << std::endl << std::endl
                  << "--externalTorquesMultiplier <multiplier>           The multiplier to scale the visualization of the external torques. Default 0.05"  << std::endl << std::endl
                  << "--externalForcesColor \"(r, g, b, [a])\"             The color used for the visualization of the external forces. Default \"(1.0, 0.0, 0.0, 1.0)\";" << std::endl << std::endl
                  << "--externalTorquesColor \"(r, g, b, [a])\"            The color used for the visualization of the external torques. Default \"(0.0, 0.0, 1.0, 1.0)\";" << std::endl << std::endl
                  << "--cameraPosition \"(px, py, pz)\"                    Camera initial position. Default \"(0.8, 0.8, 0.8)\";" << std::endl << std::endl
                  << "--imageWidth <width>                               The initial width of the visualizer window. Default 800;" << std::endl << std::endl
                  << "--imageHeight <height>                             The initial height of the visualizer window. Default 600;" << std::endl << std::endl
                  << "--maxFPS <fps>                                     The maximum frame per seconds to update the visualizer. Default 65;" << std::endl << std::endl
                  << "--backgroundColor \"(r, g, b, [a])\"                 Visualizer background color. Default \"(0.0, 0.4, 0.4, 1.0)\";" << std::endl << std::endl
                  << "--floorGridColor \"(r, g, b, [a])\"                  Visualizer floor grid color. Default \"(0.0, 0.0, 1.0, 1.0)\";" << std::endl << std::endl
                  << "--floorVisible <true|false>                        Set the visibility of the visualizer floor grid. Default true;" << std::endl << std::endl
                  << "--worldFrameVisible <true|false>                   Set the visibility of the visualizer world frame. Default true;" << std::endl << std::endl
                  << "--streamImage <true|false>                         If set to true, the visualizer can publish on a port what is rendered in the visualizer. Default true;" << std::endl << std::endl << std::endl
                  << " The following optional arguments are used only if the streaming of the image is enabled (see --streamImage):" << std::endl << std::endl
                  << "--OUTPUT_STREAM::portName <name>                   The suffix of the port where the stream the image. Default \"image\";" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::imageWidth <width>                The width of the image streamed on the port. Default 400;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::imageHeight <height>              The height of the image streamed on the port. Default 400;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::maxFPS <fps>                      The maximum number of times per second the image is streamed on the network. Default 30;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::mirrorImage <true|false>          If true, it mirrors the image before streaming it. Default false;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::useRGBA <true|false>              If true, the output image is in RGBA format. RGB otherwise. Default false;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::floorVisible <true|false>         Set the visibility of the floor grid in the streamed image. Default false;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::worldFrameVisible <true|false>    Set the visibility of the world frame in the streamed image. Default false;" <<std::endl << std::endl
                  << "--OUTPUT_STREAM::backgroundColor \"(r, g, b, [a])\"  Set the background color of the streamed image. Default \"(0.0, 0.0, 0.0, 0.0)\";" << std::endl << std::endl
                  << "--OUTPUT_STREAM::floorGridColor \"(r, g, b, [a])\"   Set the floor grid color of the streamed image. Default \"(0.0, 0.0, 1.0, 1.0)\";" << std::endl << std::endl << std::endl
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

    updateWrenchesVisualization();

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

                if (m_useRGBA)
                {
                    yarp::sig::PixelRgba& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgba*>(
                                                            m_image.getPixelAddress(width, pixelImage.height)));

                    //iDynTree specifies the pixels in [0.0, 1.0], yarp between in [0, 255]
                    pixelYarp.r = pixelImage.r * 255;
                    pixelYarp.g = pixelImage.g * 255;
                    pixelYarp.b = pixelImage.b * 255;
                    pixelYarp.a = pixelImage.a * 255;
                }
                else
                {
                    yarp::sig::PixelRgb& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgb*>(
                                                           m_image.getPixelAddress(width, pixelImage.height)));

                    //iDynTree specifies the pixels in [0.0, 1.0], yarp between in [0, 255]
                    pixelYarp.r = pixelImage.r * 255;
                    pixelYarp.g = pixelImage.g * 255;
                    pixelYarp.b = pixelImage.b * 255;
                }


            }
        }
        yarp::sig::FlexImage& imageToBeSent = m_imagePort.prepare();
        imageToBeSent.setPixelCode(m_useRGBA ? VOCAB_PIXEL_RGBA : VOCAB_PIXEL_RGB);
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
    m_netExternalWrenchesPort.close();
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

std::string idyntree_yarp_tools::Visualizer::reconnectToRobot()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    connectToTheRobot();

    if (m_connectedToTheRobot && (!m_useWBD || m_connectedToWBD))
    {
        return "[ok]";
    }
    else
    {
        if (!m_connectedToTheRobot)
        {
            return "Failed to connect to the robot!";
        }
        else
        {
            return "The connection to the robot succeeded, but not the connection to the net external wrenches port.";
        }
    }
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
