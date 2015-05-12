# Quad_Controller
**Author:** Jay Mills 19/02/2015

**Edited by:** Jeremie Bannwarth, 12/05/2015

## Introduction
This is the first version of the quadcopter controller running off vicon measurments.

The altitude controller is experimental at the moment, and does not currently have an attitude controller


## Installing
- Download install.sh and place in directory you want to download the software to
- In a terminal go to directory install.sh is and run: `./install.sh`
- The install process will take a few minutes depending on the internet connection
- The install process has been tested on Ubuntu 14.04 but should work with 13.10
- Other versions of ubuntu require different versions of ROS to run


## Running the Program
- Connect an ethernet cable to the computer running ros, make sure your ethernet IPv4 settings are as follow: `Address = 10.0.0.20`, `Netmask = 255.0.0.0`, `Gateway = 10.0.0.10`
- Run commands in seperate terminals:
  - `roscore`
  - `./run_vrpn.sh name` (for all required tracked objects)
	(where name is replaced with name of object being tracked)
  - `python quad_controller.py`
- After running `./run_vrpn_client.sh`, in a new terminal run: `rostopic echo /name/pose`. This should have a stream of data if connected properly, (name being replaced with name of object

## Problems with Running
Most known problems can be fixed by running:
`./clean_build.sh`

## Making Changes

### Making changes to the Python control program (quad_control.py)
- To input your own control algorithm, place functions into `ControlSystem.control_loop()` function
 after `self.calc_error()` call (around line 135)
- Adjust the `self.calc_error()` call to product the correct error formulations as required

### Adding in extra tracked rigid body objects:
- Define as rigid body in the main function (see rigid_body.py) as same name as in tracking software
 (around line 689)
- Call update position values inside the `ControlSystem.control_loop()` function (around line 130)

### Making changes to the UI:
- Edit ControlPannel.ui in Qt
- Run the below command to compile the ui into a python runnable file
- `./convert_ui.py ControlPannel ControlPannel`
- Adjust accordingly in UI_ControlPannel class in quadcopter_control.py
- `UI_ControlPannel.setup()`, initialises all callbacks
- `UI_ControlPannel.update()`, loops to update output values to the ui
