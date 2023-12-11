# ISS
This repo provides ROS-humble support for the ISS project, compatible with CARLA 0.9.13. The development of this repository is ongoing.

**Note**: This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.

## Install
- Install [ROS-humble](http://wiki.ros.org/humble/Installation/Ubuntu) 
- Install [CARLA 0.9.13](https://carla.readthedocs.io/en/0.9.13/start_quickstart/), also the [additional maps](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.13.tar.gz). Simply put the additional maps zip file into the `<CARLA_ROOT>/Import` folder and run `bash <CARLA_ROOT>/ImportAssets.sh` to import the maps.
- Install this repo:
```
mkdir -p ~/iss_ros2_ws/src && cd ~/iss_ros2_ws/src
git clone -b ros2-dev <this repo>
cd <this repo>
```
- ROS2 does not work well together with virtual environments, like the ones provided by Anaconda. Thus, make sure that the Python packages listed in `requirements.txt` are installed.
- Install other dependencies
```
sudo apt-get install ros-humble-teleop-twist-keyboard
```
- Build:
```
cd ~/iss_ros2_ws/ && colcon build
source install/setup.bash
```

## Run tasks
- Run CARLA server:
```
bash <CARLA root>/CarlaUE4.sh -quality-level=Low -windowed
```
- Try the simple agent build by CARLA team (in a new terminal):
```
ros2 run carla_bridge carla_bridge.launch simple_agent_demo:=true
``` 
- Try ISS agent ((in a new terminal)):
```
ros2 launch planning planning.launch
```
