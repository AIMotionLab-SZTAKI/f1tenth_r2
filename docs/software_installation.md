# Software installation and setup instructions

## Installation

### Command PC setup

The first  step is to install Ubuntu 20.04 and ROS2 Foxy on the computer. For help check out:

- [Ubuntu 20.04 installation](https://releases.ubuntu.com/focal/)
- [ROS2 Foxy installation](https://docs.ros.org/en/foxy/Installation.html)

Install <code>f1tenth_r2</code> by cloning the repostory from Github:

<code>$ git clone https://github.com/AIMotionLab-SZTAKI/f1tenth_r2 </code>
### Motion-capture configuration

First of all, the f1tenth_r2 needs to be configured with the current setup. Open the ```param.yaml``` file which can be found here: ```f1_PC/src/mocap_pkg/config/```

```
parameter_server:
  ros__parameters:
    hostname: "192.168.2.141"
    mocap_type: "optitrack"
    vehicle_id_list: ["TestName", "JoeBushJr"]

    JoeBushJr:
      IP: "192.168.2.62"
      Username: "f1tenth"
      Password: "123456"
    TestName:
      IP: "192.168.2.60"
      Username: "this_is_a_test"
      Password: "template_password"
```

#### This yaml file contains:
- The IP address of the OptiTrack server (```hostname```)
- The names of the vehicles in the fleet (```vehicle_id_list```)
- And the IP-s, usernames, and passwords for each vehicle (this will be important during the installition of the onboard stack)

Edit ```hostname```, ```vehicle_id_list``` and specify the ```IP```, ```Username``` and ```Password``` for each vehicle (as shown in the example)

When running this package, ROS2 will create a topic for each vehicle to publish the position data (```/aimotion_mocap_node/rigid_bodies/<car_id>/pose```)

### Building the environment

The next step is to set up the ROS2 workspace

You have set up the environment by sourcing ROS2 ("underlay")

```$ source /opt/ros/foxy/local_setup.bash```

---

**_Note_** To avoid the need to source the setup file each time you open a new shell, consider adding the command to your shell's startup script. This ensures the setup file is automatically executed upon shell initialization.

```$ echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc```

---

From the root of the workspace (```f1_PC/```), you have to build the packages using the command:

<code>$ colcon build</code>

In the root (```f1_PC/```) source the overlay:

<code> $ source install/setup.bash</code>

### OptiTrack server PC setup

Install the Motive software on the server PC. The software is available [here](https://optitrack.com/support/downloads/motive.html)

In this software you have to define the rigidbodies corresponding to the vehicles, with unique ID-s, that are specified in the ```f1_PC/src/mocap_pkg/config/param.yaml``` file.
### Vehicle onboard setup

The <code>f1tenth_r2</code> package takes care of the F1TENTH onboard software installation, but there are some prerequisites. ROS2 Foxy needs to be installed and additional Python packages are required by the software, which need to be installed manually. Currently, the onboard stack requires:

- ```pyyaml```
- ```numpy```
- ```scipy```

They can be installed via pip:

```
$ sudo apt install pip
$ pip install numpy scipy pyyaml
```

After configuring the environment, run the installer ```fleet_install.py``` from the scripts folder, to install the onboard stack:

```
$ cd f1_PC
$ source install/setup.bash
$ cd scripts
$ python3 fleet_install.py
```

This command will open a list of the vehicle names that are mentioned in the ```f1_PC/src/mocap_pkg/config/param.yaml```.

![installer_screenshot](/table_of_contents/pictures/installer_screenshot.jpg)

To install the onboard stack onto a specific vehicle, double click on its name. This will initiate the installation (you can track the progress in the terminal window).


---

**_Note_** Before running the script don't forget to source the workspace otherwise the installer script won't be able to access the ```param.yaml``` file.

---

### Config file description
Keep in mind, most of the subpackages include config files.
In this list, I will explain the purpose of each parameter file.

**f1_PC**:

- mocap_pkg: [_see above_](#motion-capture-configuration)

**f1_car**:

---

**_Note_** To make the change take effect, you need to rerun the installation script.

---

- vehicle_state_observer: 

```
state_observer_node:
  ros__parameters:
    FREQUENCY: 60 #The frequency of publishing the vehicle state"
    MARKER_OFFSET: 0.0 #[m] OptiTrack RigidBody center-point from the Vehicle's CoM
    
    MOCAP_EXTERNAL_TOPIC: "/aimotion_mocap_node/rigid_bodies/<id>/pose" 


    # Specifies which topic the external mocap node publishes to. (only required if motion_tracking is set to external):
    POSE_TOPIC: /aimotion_mocap_node/rigid_bodies/JoeBushJr/pose
    CUTOFF: 2.0
```

---

**_Note_** You need to change the ```POSE_TOPIC```  parameter according to the name of the vehicle. (format: ```aimotion_mocap_node/rigid_bodies/<vehicle_name>/pose```)

---
- vehicle_control:

Contains the physical parameters of the vehicle
```
control_loader_node:
  ros__parameters:
    CAR_ID: "JoeBushJr" 
    #Only used to specify the vehicle during the trajectory execution feedback

    FREQUENCY: 40.0
    LATERAL_CONTROL_GAINS:
      #k1: [-0.0000,0.0002,-0.0013,0.0225] # GS base
      #k2: [-0.0089,0.0691,-0.2403,0.8267]
      #k3: [0.0026,-0.0667,0.4328,-0.0604]
      #k1: [-0.0006,0.0051,-0.0170,0.1215] # GS high
      #k2: [-0.0494,0.3184,-0.7847,3.2132]
      #k3: [0.0350,-0.3029,1.0589,-0.1582]
      k1: [0.00266,-0.0168,0.0368,0.0357] #LPV base
      k2: [0.0424,-0.268,0.588,0.57]
      k3: [0.00952,-0.109,0.469,0.0322]

      # reversing gains
      k1_r: [-0.0008,0.0442, -1.2247] # GS
      k2_r: [-0.0002,0.0191,-0.9531]

    LONGITUDINAL_CONTROL_GAINS:
      k1: [0.0001,-0.0014,0.0908]
      k2: [-0.0025,0.0298,0.0095]
    
      # model parameters required for the feedforward
      m: 2.9 # [kg]
      C_m1: 60.0 # [N]
      C_m2: 3.0 # [Ns/m]
      C_m3: 0.6 # [N]
```

- vesc/vesc_driver (```vesc_config.yaml```):
In this  file you can limit e.g duty cycle of the motor and the streering angle
```
/**:
  ros__parameters:
    port: "/dev/ttyACM0" 
    brake_max: 200000.0
    brake_min: -20000.0
    current_max: 100.0
    current_min: 0.0
    duty_cycle_max: 0.2
    duty_cycle_min: -0.2 #This limits the duty cycle of the motor
    position_max: 0.0
    position_min: 0.0
    servo_max: 1.0 # value can be changed between [0.0, 1.0]
    servo_min: 0.0 # value can be changed between [0.0, 1.0]
    #note 0.5 is equivalent of going straight
    speed_max: 23250.0
    speed_min: -23250.0
```

---

**_Note_** If you have assigned virtual names for the USB devices you have to change the port parameter to the virtual name (**this must be done before running the installation script**)

---

### Connection between the packages:
![packages.jpg](/table_of_contents/pictures/packages.jpg)
