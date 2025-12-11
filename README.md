# UA_GIR_Tele_P04_DGGN
Code to run PHANTOM Omni + Kinova Kortex robots as a teleoperated pair with a GUI. The Omni robot is the obvious master expected to be physical, Kinova is simulated (Linux OS required)

## USE

How to start up docker and robots, step by step

First download this repo:

```cmd
git clone https://github.com/vgd10/UA_GIR_Tele_P04_DGGN.git
```

Prepare the repo for docker:

```cmd
mv UA_GIR_Tele_P04_DGGN tele
mv tele/Docker_Image\&Container_Building\&Startup/ tele/docker
mv tele/docker/Dockerfile Dockerfile
mv tele/docker/docker_run.sh docker_run.sh
mv tele/docker/Phanthom.tar Phanthom.tar
tar -xzvf Phanthom.tar Phanthom
mv tele/tests tests
rm Phanthom.tar
rm -rf tele
```

And allow your docker daemon to access the server of host computer, so GUI appears:

```cmd
xhost +
```


### Build docker image and container

Build the docker image (using Dockerfile and unzipped Phantom files) and start or create the container (with docker_run.sh, current folder where it is run would be mounted on path /catkin_ws/shared_folder in the container environment)

<br/>
Run this on a folder with Dockerfile, docker_run.sh and the folders Phantom (unzipped) and tests:

```cmd
docker build -t kinova-phantom .
```
Create or startup the container (in the same path where image was created) (kinova-phanthom_container will be created or resumed):

```cmd
./docker_run.sh
```

<br/><br/>
If you want to open more terminals (first one for personal computer, second one for labs):

```cmd
sudo docker exec -it kinova-phanthom_container /bin/bash
```


```cmd
docker exec -it kinova-phanthom_container /bin/bash
```
<br/>

### Prepare kinova robot control

To launch the kinova robot (go to folder then start node) (remember to close the other pop up joint control GUI):
<br/>

```cmd
cd /catkin_ws/kinova/
source install/setup.bash
```

```cmd
ros2 launch kortex_description view_robot.launch.py robot_type:=gen3 dof:=7 gripper:=robotiq_2f_140  use_sim_time:=true launch_rviz:=true
```
<br/><br/>
Go to kinova control files (in 3 different terminals)

```cmd
cd /catkin_ws/shared_folder/tests/
```
Then for one of each terminal run one of the commands in order (second one will terminate)

```cmd
python3 ee_cartesian_velocity_controller.py
```

```cmd
python3 home_kinova.py
```

```cmd
python3 cartesian_key_teleop.py
```
<br/><br/>

### Prepare phantom robot

Open another terminal and go to phantom folder:

```cmd
cd catkin_ws/phantom
```
Do this connection for communication (first one for USB, second one for ethernet)

```cmd
./start_omni_USB.sh
```

```cmd
./start_omni_LAN.sh
```

Then check that connect is fine:

```cmd
./Touch_Diagnostic
```

Run the haptic sensor as another check with:

```cmd
ros2 run sensable_phantom sensable_phantom
```

<br/> <br/>
Finally, run the phanthom base to have the true robot connnection:

```cmd
source install/setup.bash
ros2 launch sensable_phantom_models omni_rviz.launch.py
```

<br/><br/>

### Building workspace of the project

Create folders for workspace and package (prepared to create package)

```cmd
mkdir -p ros2_telep04_ws/src && cd ros2_telep04_ws/src
```

Create the package with nothing but dependencies and compile it (prepared to modify setup files)

```cmd
ros2 pkg create telep04_proy --build-type ament_python --dependencies rclpy geometry_msgs visualization_msgs && cd .. && colcon build --symlink-install && cd src/telep04_proy
```
[Considering current path is where setups are] Prepare setup.py file with both nodes <br/>
[**WARNING**: First head command could need to be set as -n 22, depending of ros2 jazzy build, maybe. Variable after license varies format]

```cmd
head -n 26 setup.py > setup2.py && echo "            'gravityWell = telep04_proy.gravityWell:main'," >> setup2.py && echo "            'esfera_rviz = telep04_proy.esfera_rviz:main'," >> setup2.py && tail -n 3 setup.py >> setup2.py && mv setup2.py setup.py
```
 Prepare setup.cfg file with both nodes

```cmd
echo "[options.entry_points]" >> setup.cfg && echo "console_scripts = " >> setup.cfg && echo "    gravityWell = telep04_proy.gravityWell:main" >> setup.cfg && echo "    esfera_rviz = telep04_proy.esfera_rviz:main" >> setup.cfg
```
[Source code expected to be in the original path where this started] Copy source code of the nodes, compile and install the workspace

```cmd
cd telep04_proy && cp /catkin_ws/shared_folder/gravityWell.py . && cp /catkin_ws/shared_folder/esfera_rviz.py . && cd /ros2_telep04_ws && colcon build --symlink-install && source install/setup.bash
```
<br/>

## SESSION PROGRESS

a.k.a The amazing adventures of the teleoperation group 3

### S1 (Breaking the ice session)

- Set up the container, try the basic software (see Manual_de_Uso__Control_del_Kinova_Gen3_en_Docker.pdf)

Image was built on labs computers and personal computers. To save time, the image itself has been saved on 2 USB (contributors vgd10 and nolandius777) once built on labs computer and then it was passed to personal computer.

To launch the container using docker_run.sh, lab computer required RUNTIME parameter of the file to be set as "", personal computer needed nvidia toolbox installation (see previous episode P03, currently unavailable)

Furthermore, only lab computers kept making progress with the next steps of trying the software (seemingly personal computer could launch the container at the end). All steps could be completed, although dependecies installation have been forgotten (use ./install.sh or the current test folder on this repository + pip3 install pygame)

#### Other insight gained

- GUI asked would be very simple (either text or 2D representation)
- Maximum grade without GUI is 9. Once done a GUI, an extra point is granted if robot and GUI running computers are not the same
- Maybe the files on test folder are made by other students (suspicious of home_kinova.py, self.arm_home initialization comment)
- Simulated dynamic objects would be first spheres (using the knowledge of P03 at reverse), then cubes can be tried

### S2 (S1 never happened...)

- Get a roadmap to approach the problem

#### ROADMAP PLANNED

- Subgroup 1 (ekaitzduque & vgd10): Get the kinova robot moving by topics

- Subgroup 2 (nolandius777 PabloGorbaran): Create the markers and force simulation of rigid spheres

- Pending: Calculate escale transformation between phantom and kinova, make the communication between them and do the needed changes to set it all

#### Insight gained

- Subgroup 1: Topic used tool_twist_vel. Sets a velocity for next step. Requires substraction of current and previous position of phantom, division by time passed and then escalate it to kinova.

- Subgroup 2: ???

### S3 (Frozen solid)

- No advancements done in previous session time gap. Same objectives

#### Insight gained

- Nothing at all, but github page readme has improved a lot
