# Quad_Controller
**Author:** Jay Mills 19/02/2015

**Edited by:** Jeremie Bannwarth, 12/05/2015

## Introduction
This is the first version of the quadcopter controller running off Vicon measurements.

The altitude controller is experimental at the moment, and does not currently have an attitude controller

## Installing
The install.sh bash script contains all the commands required to install this software. You can either run the bash script or copy the commands one by one in the terminal. **I recommend running the commands one by one in the terminal, as it will make debugging easier.**

If you want to install the software using the bash script:
- Download install.sh and place in directory you want to download the software to
- In a terminal go to directory install.sh is and run: `./install.sh`
- The install process will take a few minutes depending on the Internet connection
- The install process has been tested on Ubuntu 14.04 but should work with 13.10
- Other versions of Ubuntu require different versions of ROS to run


## Running the Program
- Connect an Ethernet cable between your computer and the computer running Vicon Tracker. As of June 2015, use the free ethernet port on the PC's networking card
- Make sure your Ethernet IPv4 settings are as follow: `Address = 10.0.0.20`, `Netmask = 255.0.0.0`, `Gateway = 10.0.0.10` (click on the connection icon in the Ubuntu menu bar - it can look like a wireless signal icon or two arrows depending on whether your wireless adaptor is enabled or not -, click on Edit connections, choose your ethernet connection and click Edit, click on the Edit IPv4 settings tab, set the connection mode to manual and set the Address, Netmask and Gateway settings accordingly)
- Use Vicon Tracker to start tracking an object (when an object is added in Vicon Tracker it starts being streamed automatically using the VRPN protocol):
  - Run vicon according to instructions
  - Place the object in the cameras' field of view
  - Select the markers on the screen associated with the object (ctrl+left click)
  - In the "Object" pane, at the bottom, enter a name for the object and click "create"
- Run commands in separate terminals:
  - `roscore`
  - `./run_vrpn_client.sh name` (for all required tracked objects)
	(where name is replaced with name of object being tracked)
  - `python quad_controller.py`
- After running `./run_vrpn_client.sh`, in a new terminal run: `rostopic echo /name/pose`. This should have a stream of data if connected properly (name being replaced with name of object)

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
- Edit ControlPannel.ui in Qt Designer
- Run the below command to compile the ui into a python runnable file
- `./convert_ui.sh ControlPannel ControlPannel`
- Adjust accordingly in UI_ControlPannel class in quadcopter_control.py
- `UI_ControlPannel.setup()`, initialises all callbacks
- `UI_ControlPannel.update()`, loops to update output values to the ui
