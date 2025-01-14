# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Set the environment to be non-interactive (for any prompt during the installation)
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH

# Update the package list and install prerequisites
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    curl \
    wget \
    git \
    ca-certificates \
	lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add deadsnakes PPA (for Python 3.9)
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.9 python3.9-distutils python3.9-venv python3.9-dev && \
    rm -rf /var/lib/apt/lists/*

# Install pip for Python 3.9
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3.9 get-pip.py && \
    rm get-pip.py

# Set python3.9 and pip as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    update-alternatives --install /usr/bin/pip3 pip3 /usr/local/bin/pip3 1

# Verify Python and pip installation
RUN python3 --version && pip3 --version

## Installing Cmake 3.24
RUN apt-get remove cmake -y && \
	apt-get install -y wget apt-transport-https && \
	wget https://cmake.org/files/v3.24/cmake-3.24.2-linux-x86_64.sh && \
	mkdir /opt/cmake && \
	bash cmake-3.24.2-linux-x86_64.sh --prefix=/opt/cmake --skip-license && \
	echo 'export PATH=/opt/cmake/bin:$PATH' >> ~/.bashrc

# Update PATH to include CMake
ENV PATH=/opt/cmake/bin:$PATH

# Verify the installed version of cmake
RUN cmake --version

# Setting up the source keys for ROS
RUN apt update && apt upgrade -y && \
	export PATH=/opt/cmake/bin:$PATH && \
	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	apt install -y curl && \
	apt install -y gnupg2 gnupg-agent lsb-release && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	apt update && apt install -y ros-noetic-desktop-full && \
	apt-get update && \
	apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    rm -rf /var/lib/apt/lists/* && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash"

# Fixing the issue with the netifaces package
RUN python3 -m pip install --target=/opt/ros/noetic/lib/python3/dist-packages netifaces

# Initialize rosdep
RUN rosdep init && rosdep update

# Install and use catkin
RUN apt-get update && \
	export PATH=/opt/cmake/bin:$PATH && \
	apt-get install -y python3-catkin-tools ros-noetic-catkin python3-osrf-pycommon && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash" && export PATH=$PATH:/opt/ros/noetic/bin && \
	mkdir -p ~/catkin_ws/src && \
	cd ~/catkin_ws && \  
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.9 -DPYTHON_INCLUDE_DIR=/usr/include/python3.9 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.9.so" && \
	echo "source devel/setup.bash" >> ~/.bashrc && \
	/bin/bash -c "source devel/setup.bash"

# Setting up libfcl correctly
RUN apt-get remove --purge libfcl-dev -y && \
	apt-get install -y libeigen3-dev && \
	mkdir ~/packages && cd ~/packages && \
	git clone -b 0.6.1 https://github.com/flexible-collision-library/fcl.git && \
	cd fcl && \
	mkdir build && cd build && \
	cmake .. -DCMAKE_BUILD_TYPE=Release && \
	make -j$(nproc) && \
	make install && \
	ls /usr/local/lib | grep libfcl && \
	echo "/usr/local/lib" |  tee /etc/ld.so.conf.d/fcl.conf && \
	ldconfig
	
# Install all the dependencies necessary for Moveit
RUN export PATH=$PATH:/opt/ros/noetic/bin && \
	export PATH=/opt/cmake/bin:$PATH && \
	python3 -m pip install numpy && \
	apt-get install -y ros-$ROS_DISTRO-universal-robots && \
	cd ~/catkin_ws/src && \
	git clone https://github.com/ros-industrial/ur_modern_driver.git && \
	cd ur_modern_driver/ && \
	git checkout kinetic-devel && \
	cd ../ && \
	git clone https://github.com/ros-controls/ros_control.git && \
	git clone https://github.com/ros-industrial/industrial_core.git && \
	apt install -y ros-noetic-catkin python3-catkin-tools && \
	apt install -y python3-wstool && \
	git clone https://github.com/ros-planning/moveit_tutorials.git -b master && \
	git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel && \
	git clone https://github.com/ros-planning/moveit.git && \
	cd moveit/ && \
	git checkout noetic-devel && \
	cd ../ && \
	git clone https://github.com/ros-industrial/universal_robot.git && \
	cd universal_robot/ && \
	git checkout kinetic-devel && \
	cd ../ && \
	rosdep install -y --from-paths . --ignore-src --rosdistro noetic && \
	pip install setuptools==58.0.4 && \
	pip install testresources && \
	cd ../ && \
	python3 -m pip install --target=/opt/ros/noetic/lib/python3/dist-packages numpy && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" && \
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Giving GUI access to Docker
RUN  apt-get install -y mesa-utils

# Installing MoveIt Calibration
RUN cd ~/catkin_ws/src && \
	export PATH=/opt/cmake/bin:$PATH && \
	git clone https://github.com/moveit/moveit_calibration.git && \
	rosdep update && rosdep install -y --from-paths . --ignore-src --rosdistro noetic && \
	apt-get install --reinstall python-numpy && cd .. && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" && \
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	/bin/bash -c "source devel/setup.bash" && \
	apt-get install ros-noetic-ur5-moveit-config

# Setting up the Launch files and configuring MoveIt
RUN cd ~/catkin_ws/src/universal_robot/ur5_moveit_config && \
	apt-get update &&  apt-get install -y ros-noetic-realsense2-camera && \
	echo '<?xml version="1.0"?>' > robot_realsense.launch && \
    echo '<launch>' >> robot_realsense.launch && \
    echo '    <arg name="namespace_prefix" default="robot_realsense" />' >> robot_realsense.launch && \
    echo '    <arg name="robot_ip" doc="The IP address of the UR5 robot" default="192.10.0.11" />' >> robot_realsense.launch && \
    echo '' >> robot_realsense.launch && \
    echo '    <!-- start the realsense -->' >> robot_realsense.launch && \
    echo '    <include file="$(find realsense2_camera)/launch/rs_rgbd.launch" >' >> robot_realsense.launch && \
    echo '        <arg name="color_height" value="720" />' >> robot_realsense.launch && \
    echo '        <arg name="color_width" value="1280" />' >> robot_realsense.launch && \
    echo '        <arg name="color_fps" value="30" />' >> robot_realsense.launch && \
    echo '    </include>' >> robot_realsense.launch && \
    echo '' >> robot_realsense.launch && \
    echo '    <!-- start the robot -->' >> robot_realsense.launch && \
    echo '    <include file="$(find ur_modern_driver)/launch/ur5_bringup.launch">' >> robot_realsense.launch && \
    echo '        <arg name="limited" value="true" />' >> robot_realsense.launch && \
    echo '        <arg name="robot_ip" value="$(arg robot_ip)" />' >> robot_realsense.launch && \
    echo '    </include>' >> robot_realsense.launch && \
    echo '' >> robot_realsense.launch && \
    echo '    <!-- start the motion planning -->' >> robot_realsense.launch && \
    echo '    <include file="$(find ur5_moveit_config)/launch/ur5_moveit_planning_execution.launch">' >> robot_realsense.launch && \
    echo '        <arg name="limited" value="true" />' >> robot_realsense.launch && \
    echo '    </include>' >> robot_realsense.launch && \
    echo '</launch>' >> robot_realsense.launch && \
	rosdep update && rosdep install -y --from-paths . --ignore-src --rosdistro noetic && \
	export PATH=/opt/cmake/bin:$PATH && \
	cd ~/catkin_ws && /bin/bash -c "source devel/setup.bash && catkin_make" && \
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install Intel Realsense SDK - realsense2
RUN apt-get -y update &&  apt-get -y upgrade &&  apt-get -y dist-upgrade && \
	apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y && \
	apt-get install git wget build-essential -y && \
	apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y && \
	apt install v4l-utils -y && \
	apt-get install usbutils -y && \
	apt-get install build-essential python3-dev python3-pip python3-setuptools python3-numpy cython3 -y && \
	pip install --upgrade importlib-metadata && pip install --upgrade setuptools && \
	pip install pyrealsense2 && pip install opencv-python && \
	apt-get install autoconf libudev-dev -y && \
	cd ~/packages && \
	git clone https://github.com/Microsoft/vcpkg.git && \
	cd vcpkg && \
	./bootstrap-vcpkg.sh && \
	./vcpkg integrate install && \
	./vcpkg install realsense2

# Install Intel Realsense SDK - librealsense
RUN	apt-get -y update &&  apt-get -y upgrade &&  apt-get -y dist-upgrade && \
	mkdir -p /etc/apt/keyrings && \
	curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp |  tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
 	apt-get install apt-transport-https -y && \
	echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
	tee /etc/apt/sources.list.d/librealsense.list && \
	apt-get update && \
	apt-get install librealsense2-dkms -y && \
	apt-get install librealsense2-utils -y && \
	apt-get install librealsense2-dev -y && \
	apt-get install librealsense2-dbg -y && \
	apt-get update &&  apt-get upgrade -y

# Verify the installation of the Realsense SDK
RUN realsense-viewer --version

# Setting up rgbd_launch package
RUN apt-get update && apt-get install ros-noetic-rgbd-launch -y && \
	echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/ros/noetic/share/rgbd_launch' >> ~/.bashrc

# Building Open3D from source for verifying the camera calibration
RUN export PATH=/opt/cmake/bin:$PATH && \
	cd ~/packages && git clone https://github.com/isl-org/Open3D && \
	cd Open3D && yes | util/install_deps_ubuntu.sh && \
	mkdir build && cd build && cmake .. && \
	make -j$(nproc) && make install && \
	make install-pip-package

# Fixing issues with numpy installation occurred in the setup steps above
# RUN rm -rf /opt/ros/noetic/lib/python3/dist-packages/numpy* && \
# 	yes | sudo apt-get remove --purge python3-numpy && \
# 	rm -rf /usr/lib/python3/dist-packages/numpy* && rm -rf ~/.local/lib/python3*/site-packages/numpy* && \
# 	pip install numpy --upgrade && \
# 	export CMAKE_PREFIX_PATH=/opt/ros/noetic:$CMAKE_PREFIX_PATH && \
# 	rosdep install --from-paths src --ignore-src -r -y && \
# 	cd ~/catkin_ws && /bin/bash -c "source devel/setup.bash && catkin_make clean" && \
# 	/bin/bash -c "source devel/setup.bash && catkin_make" && \
# 	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container
COPY . /app

# Make sure to source bashrc regardless of how the container is started
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]