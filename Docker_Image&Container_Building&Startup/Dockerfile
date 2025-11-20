FROM osrf/ros:jazzy-desktop-noble

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Install necessary libraries for GUI applications
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo

#
RUN mv /usr/lib/python3.12/EXTERNALLY-MANAGED /usr/lib/python3.12/EXTERNALLY-MANAGED.old

# tools
RUN apt-get update && apt-get -y install vim udev usbutils nano
#RUN apt install -y python3-wstool python3-catkin-tools
RUN apt-get -y install python3-pip  libusb-1.0-0-dev
RUN python3 -m pip install pyserial

# Home 
RUN mkdir -p /catkin_ws/kinova &&\
    mkdir -p /catkin_ws/phanthom
    
# Copy dependeces for phanthom
RUN mkdir -p /catkin_ws/phanthom/dependences/Phanthom
COPY /Phanthom /catkin_ws/phanthom/dependences/Phanthom
COPY ./Phanthom/99-3dsystems.rules /etc/udev/rules.d/
RUN cd /catkin_ws/phanthom/dependences/Phanthom && cp libPhantomIOLib42.so /usr/lib/
RUN cd /catkin_ws/phanthom/dependences/Phanthom/openhaptics_3.4-0-developer-edition-amd64 && chmod +x install && ./install 

# phanthom dependeces
RUN apt install -y libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev
RUN apt-get install -y freeglut3-dev libncurses5-dev zlib1g-dev
RUN apt-get install -y libncursesw5-dev #libncurses5:i386
RUN apt-get install -y libfltk1.3-dev fluid 

# Kinova Kortex repository
#RUN apt-get update && apt-get install -y python3-full python3-pip
#RUN echo "cython<3" > /tmp/constraint.txt
#RUN PIP_CONSTRAINT=/tmp/constraint.txt python3 -m pip install conan==1.59 --ignore-installed PyYAML
#COPY ./Phanthom/loader.py /usr/local/lib/python3.12/dist-packages/conans/client/loader.py 
#RUN conan config set general.revisions_enabled=1
#RUN conan profile new default --detect > /dev/null
#RUN conan profile update settings.compiler.libcxx=libstdc++11 default
#RUN mkdir -p /catkin_ws/kinova/src && cd /catkin_ws/kinova/src && git clone https://github.com/Kinovarobotics/ros2_kortex.git && cd ros2_kortex && git reset --hard 309f9c9
#RUN apt install -y python3-rosdep
#RUN cd /catkin_ws/kinova && rosdep update && rosdep install --from-paths src --ignore-src -y

RUN apt-get update && apt-get install -y python3-full python3-pip
RUN apt install -y ros-jazzy-rmw-cyclonedds-cpp
RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN apt install -y python3-colcon-common-extensions python3-vcstool
RUN mkdir -p /catkin_ws/kinova/src && cd /catkin_ws/kinova/src && git clone https://github.com/Kinovarobotics/ros2_kortex.git
RUN cd /catkin_ws/kinova && vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.jazzy.repos
RUN cd /catkin_ws/kinova && vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.jazzy.repos
RUN cd /catkin_ws/kinova && vcs import --skip-existing --input src/ros2_kortex/simulation.repos
RUN apt install -y python3-rosdep
RUN cd /catkin_ws/kinova && rosdep update && rosdep install --from-paths src --ignore-src -y

RUN cd /catkin_ws/kinova && rm -rf gz* && rm -rf ros*

# Catkin make Kinova Kortex
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash &&\
    cd /catkin_ws/kinova &&\
    colcon build --parallel-workers $(nproc)' 

# create workspace for phanthom
RUN cd /catkin_ws/phanthom && mkdir src
# Clone repository for phanthom
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/jhu-cisst/cisstNetlib.git && cd cisstNetlib && git reset --hard 1978a78 
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/jhu-cisst/cisst.git && cd cisst && git reset --hard afb84bf 
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/collaborative-robotics/crtk_msgs.git && cd crtk_msgs && git reset --hard a7f6375
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/collaborative-robotics/crtk_python_client.git && cd crtk_python_client && git reset --hard 1e34368
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/jhu-cisst/cisst-ros.git && cd cisst-ros && git reset --hard c182467
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/jhu-saw/sawKeyboard.git --branch 1.4.0
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/jhu-saw/sawControllers.git && cd sawControllers && git reset --hard c3afb67
RUN wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb && wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libncurses5_6.3-2ubuntu0.1_amd64.deb && apt install ./libtinfo5_6.3-2ubuntu0.1_amd64.deb ./libncurses5_6.3-2ubuntu0.1_amd64.deb
RUN cd /catkin_ws/phanthom/src && git clone https://github.com/jhu-saw/sawSensablePhantom.git && cd sawSensablePhantom && git reset --hard 7ddd575

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /catkin_ws/phanthom && \
    colcon build --parallel-workers $(nproc)"

# Install ufw Firewall tools
RUN apt-get update && apt-get install ufw -y 

# bash update
RUN echo "TERM=xterm-256color" >> ~/.bashrc
RUN echo "# COLOR Text" >> ~/.bashrc
RUN echo "PS1='\[\033[01;33m\]\u\[\033[01;33m\]@\[\033[01;33m\]\h\[\033[01;34m\]:\[\033[00m\]\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc
RUN echo "CLICOLOR=1" >> ~/.bashrc
RUN echo "LSCOLORS=GxFxCxDxBxegedabagaced" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "## ROS" >> ~/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Establecer variables de entorno para NVIDIA
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

RUN cd /catkin_ws/phanthom/dependences/Phanthom && chmod 777 Touch_Setup

COPY ./Phanthom/start_omni_LAN.sh /catkin_ws/phanthom/start_omni_LAN.sh
COPY ./Phanthom/start_omni_USB.sh /catkin_ws/phanthom/start_omni_USB.sh
COPY ./Phanthom/Touch_Diagnostic /catkin_ws/phanthom/Touch_Diagnostic
COPY ./Phanthom/Touch_Setup /catkin_ws/phanthom/Touch_Setup
RUN chmod +x /catkin_ws/phanthom/start_omni_USB.sh /catkin_ws/phanthom/start_omni_LAN.sh /catkin_ws/phanthom/Touch_Diagnostic /catkin_ws/phanthom/Touch_Setup

# apt install x11-apps -> xeyes o xclock
