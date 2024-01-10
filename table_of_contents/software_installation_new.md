# Software installation and setup instructions


## Command PC setup

The first  step is to install Ubuntu 20.04 and ROS2 Foxy and the crazymocap package on the computer. For help check out:

- [Ubuntu 20.04 installation](https://releases.ubuntu.com/focal/)
- [ROS2 Foxy installation](https://docs.ros.org/en/foxy/Installation.html)
- [Crazymocap](https://github.com/AIMotionLab-SZTAKI/crazymocap)

Install <code>f1tenth_r2</code> by cloning the repostory from Github:

<code>$ git clone https://github.com/AIMotionLab-SZTAKI/f1tenth_r2 </code>
<!--- 
### Crazymocap configuration

We are utilizing the crazymocap package for streaming low-latency motion-capture data through the Bitcraze Crazyradio. To specify the vehicle's name, you'll need to modify the ```obj_name``` parameter within the constructor of the RadioStreamer class in the radio_streamer.py file. Additionally, ensure the devid parameter is set to 0.

```
if __name__=="__main__":
    print("Starting TX")
    streamer = RadioStreamer(obj_name="cf2", devid=1)
    try:
        while True:
            streamer.send_pose()
    except Exception as exc:
        print(f"Exception: {exc!r}. TRACEBACK:\n")
        print(traceback.format_exc())
        streamer.close()
```

-->

## Using the fleet manager

The ```f1tenth_r2``` package comes with a fleet manager ui, which provides the following services:
- Adding new vehicle to the existing fleet
- Removing vehicles from the fleet
- Editing parameters of the vehicles
- [Installing the onboard stack to any vehicle in the fleet](#installation-of-the-onboard-stack)
- Editing the login parameters of the vehicles *(the login data is used during the installation of the onboard stack)*

To run the fleet manager use the following command in your terminal window:

```
$ cd f1_PC/scripts
$ python3 ui.py
```


![installer_screenshot](/table_of_contents//pictures/ui_installer_main.png)

### Requirements before installation

Before installing the ```f1tenth_r2``` package on the vehicle, ensure the following prerequisites are met:

- ROS2 Foxy: Make sure ROS2 Foxy is installed on the vehicle's system.

- Additional Python packages: Manually install the following Python packages required by the onboard stack on the vehicle:

    - pyyaml
    - numpy
    - scipy

The Python packages can be installed via pip:

```
$ sudo apt install pip
$ pip install numpy scipy pyyaml
```

The f1tenth_r2 package manages the installation of F1TENTH onboard software on the vehicle. However, it's crucial to have ROS2 Foxy installed along with the specified Python packages for the proper functioning of the onboard stack.

After installing the necessary Python packages, follow these steps:
- Open ```/etc/udev/rules.d/99-vesc.rules``` as the root user
- Copy and paste the following rule for the VESC into the file:
```
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"
```

- Finally, trigger (activate) the rule by running:
```
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

---

**_Note_** You can check your VESC's id by listing your connected USB devices:
```
$ usb-devices
```

---
### Installation of the onboard stack




Firstly, add the new vehicle to the fleet by clicking on the 'Add new vehicle' button. Enter the vehicle's name into the textbox and click 'OK'.

Once added, your vehicle's name will appear in the list. Before initiating the installation, specify the login information. Select your vehicle and click on the 'Login edit' button. In the subsequent window, enter the vehicle's IP address along with your username and password.

Finally, to install the onboard stack, select your vehicle and click the 'Install' button. This action will open a new window where you can track the progress of the installation.

### Usage of the ```f1tenth/f1_PC``` packages
In this subpackage every message type used by the vehicle is stored. 
To echo the messages published to the running topics on the vehicle, you'll need to build this package on the server PC (or on any other computer) and source the environment.
```
$ cd f1_PC
$ colcon build
$ source install/setup.bash
```

After that you will be able to listen to any ROS2 topic. 

### Running crazymocap:

For localization, the ```crazymocap``` radio streamer is used. On the server PC run the following command:

```
$ python3 crazymocap/radio_streamer.py -i <mocap_server_ip> -o <vehicle_name>
```

---

**_Note_** You can set your default ```<mocap_server_ip>``` and ```<vehicle_name>```. To do this open the ```radio_streamer.py``` script.

```

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--ip",  default= "192.168.2.141")
parser.add_argument("-o", "--object_name",  default= "JoeBush1")
parser.add_argument("-d", "--devid", default = 0)
args = vars(parser.parse_args())

```


---
### Connection between the packages:
![connection_diagram](/table_of_contents/pictures/Untitled%20Diagram.drawio.png)
