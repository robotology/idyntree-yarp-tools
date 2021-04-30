service VisualizerCommands
{
    /**
     * Set the base position.
     * The values are expected in meters.
     * @return true/false in case of success/failure.
     */
    bool setBasePosition(1:double x, 2:double y, 3:double z);

    /**
     * Set the base rotation.
     * The values are expected in degrees.
     * @return true/false in case of success/failure.
     */
    bool setBaseRotation(1:double roll, 2:double pitch, 3:double yaw);

    /**
     * Set the base pose.
     * The x, y, and z values are expected in meters
     * The roll, pitch, and yaw avalues are expected in degrees.
     * @return true/false in case of success/failure.
     */
    bool setBasePose(1:double x, 2:double y, 3:double z, 4:double roll, 5:double pitch, 6:double yaw);

    /**
     * Attempt to reconnect to the robot.
     * @return A status message. "[ok]" in case of success
     */
    string reconnectToRobot();

    /**
     * Get the camera position
     * @return A vector of 3 elements with the camera position.
     */
    list<double> getCameraPosition();

    /**
     * Get the camera target
     * @return A vector of 3 elements with the camera target.
     */
    list<double> getCameraTarget();
}
