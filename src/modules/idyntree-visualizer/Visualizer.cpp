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

    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
    m_modelLoader.loadReducedModelFromFile(pathToModel, m_jointList);

    m_viz.init(m_options);
    m_textureInterface = m_viz.textures().add("AdditionalTexture", m_textureOptions);

    m_viz.camera().setPosition(iDynTree::Position(1.2, 0.0, 0.5));
    m_viz.camera().setTarget(iDynTree::Position(-0.15, 0.0, 0.15));
    m_viz.camera().animator()->enableMouseControl(true);

    double sqrt2 = std::sqrt(2.0);
    m_viz.enviroment().lightViz("sun").setDirection(iDynTree::Direction(0.5/sqrt2, 0, -0.5/sqrt2));
    m_viz.enviroment().addLight("secondSun");
    m_viz.enviroment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    m_viz.enviroment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));

    m_textureInterface->environment().lightViz("sun").setDirection(iDynTree::Direction(0.5/sqrt2, 0, -0.5/sqrt2));
    m_textureInterface->environment().addLight("secondSun");
    m_textureInterface->environment().lightViz("secondSun").setType(iDynTree::LightType::DIRECTIONAL_LIGHT);
    m_textureInterface->environment().lightViz("secondSun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));
    m_textureInterface->environment().setElementVisibility("floor_grid", false);
    m_textureInterface->environment().setElementVisibility("world_frame", false);
    m_textureInterface->environment().setBackgroundColor(iDynTree::ColorViz(0.0, 0.0, 0.0, 0.0));

    m_viz.addModel(m_modelLoader.model(), "robot");

    m_image.setPixelCode(VOCAB_PIXEL_RGB);
    m_image.resize(m_textureOptions.winWidth, m_textureOptions.winHeight);

    if (!m_offline)
    {
        std::string rpcPortName = "/visualizer/rpc";
        this->yarp().attachAsServer(this->m_rpcPort);
        if(!m_rpcPort.open(rpcPortName))
        {
            yError() << "Could not open" << rpcPortName << " RPC port.";
            return false;
        }
        m_imagePort.open("/visualizerImage");
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
    while(m_viz.run() && !m_isClosing)
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

    if (!m_offline && std::chrono::duration_cast<std::chrono::microseconds>(m_now - m_lastSent).count() >= m_minimumMicroSec)
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
