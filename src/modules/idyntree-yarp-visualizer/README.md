# idyntree-yarp-visualizer

This module allows visualing a robot starting from its URDF. 

![image](https://user-images.githubusercontent.com/18591940/115052071-cda47400-9edd-11eb-962a-c136abf2bfdb.png)

## How to run
The module relies on a set of parameters. For each of them, a default value is specified, so the module can simply run with the command

```
idyntree-yarp-visualizer
```
By default, it looks for the robot URDF model with name ``model.urdf``, searched according to the ``YARP_ROBOT_NAME``. It will then trying to connect to the robot according to the list of joints reported in the model.

The configuration of the visualizer can be changed via configuration file or by command line arguments. Run the following command to have the full list of parameters and their default
```
idyntree-yarp-visualizer --help
```

### Ports opened

By default, the ``idyntree-yarp-visualizer``  opens two ports

- A RPC port for a basic interaction with the visualizer. By default, the port name is ``/idyntree-yarp-visualizer/rpc``.  Use the command ``help`` to get a list of possible commands. Run ``help <commandName>`` to get informations about a specific command.
```
yarp rpc /idyntree-yarp-visualizer/rpc
>>help
```
- A port streaming the visualization output. By default, this port is named ``/idyntree-yarp-visualizer/image``, and allows to see the visualizer output remotely.

According to the method adopted to connect to the robot, the module will open other ports for internal use.
