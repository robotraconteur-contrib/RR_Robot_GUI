# Robot Raconteur (RR) Simple Robot Client

This will display the three images side by side if the images are not too wide.

<p float="left">
  <img src="/images/gui.png" width="150" />
  <img src="/images/gui2.png" width="800" /> 
</p>
## Prerequisite:
* python2/python3 (`python` for python2, `python3` for python3, be aware of which one you're using)
* [qpsolvers](https://pypi.org/project/qpsolvers/) (`pip/pip3 install qpsolvers`)
* [numpy](https://pypi.org/project/numpy/) (`pip/pip3 install numpy`)
* [importlib](https://pypi.org/project/importlib/) (`pip/pip3 install importlib`)
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur/wiki/Download) (Follow instruction to download, depending on different OS)
* [RPI Robotics Toolbox](https://github.com/rpiRobotics/general-robotics-toolbox/tree/master/Python) (already included under `/toolbox`)


## Instructions:
Start RR robot service, and make sure it's exposed to the network (checking by [RR Service Browser](https://github.com/robotraconteur/RobotRaconteur_ServiceBrowser))

Create robot orientation function under `\toolbox` if robot not included in the provided ones, named it `R_<robot_name>`.

Start the script:
`python gui_client_robot.py --robot-name=<robot_name>`.

The script will detect robot service through auto-discovery based on provided robot name, and also connecting to the tool service if available.
