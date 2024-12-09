# idyntree-yarp-tools
Tools based on the iDynTree library that depend on YARP.

## Available tools

### `idyntree-yarp-visualizer`

![image](https://user-images.githubusercontent.com/18591940/115052071-cda47400-9edd-11eb-962a-c136abf2bfdb.png)

Module to visualize a robot using the iDynTree Visualizer library, loading the robot description from URDF file, and reading its state via YARP ports and devices. Check detailed docs in [`idyntree-yarp-tools` README](src/modules/idyntree-yarp-visualizer/README.md).

### `yarprobotstatepublisher`

Drop-in replacement of the [ROS `robot_state_publisher`](http://wiki.ros.org/robot_state_publisher) that uses iDynTree for loading the URDF
and computing the forward kinematics, and YARP to read the joint positions and publish the computed transforms.

### `urdf2dh`

Command-line tool to convert chains extracted from a [URDF model](http://wiki.ros.org/urdf)  to [Denavit-Hartenberg (DH) parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters).

The output DH parameters are saved in a `.ini` file that can be directly loaded in the [iKin library](https://robotology.github.io/robotology-documentation/doc/html/group__iKin.html), part of the [`icub-main`](https://github.com/robotology/icub-main) project.

### `idyntree-plotter`

[Qt Charts](https://doc.qt.io/qt-5/qtcharts-index.html)-based tool that can receive plot commands from YARP RPC commands. The protocol of the RPC command is specified in the [`src\modules\idyntree-plotter\thrifts\chartsService.thrift`](src\modules\idyntree-plotter\thrifts\chartsService.thrift) YARP Thrift file.

### `idyntree-sole-gui`

[Qt](https://www.qt.io/)-based tool to plot soles boundaries projected on a 2D contact plain, and relevant points.


Maintainers
--------------
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/S-Dafarra.png" width="40">](https://github.com/S-Dafarra) | [@S-Dafarra](https://github.com/S-Dafarra) |
| [<img src="https://github.com/traversaro.png" width="40">](https://github.com/traversaro) | [@traversaro](https://github.com/traversaro) |
