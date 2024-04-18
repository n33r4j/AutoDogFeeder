FROM arm64v8/ros:iron

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt-get upgrade -y

# Install Git, Nano and other packages (https://forums.docker.com/t/dockerfile-run-apt-get-install-all-packages-at-once-or-one-by-one/17191)
RUN sudo apt-get install -y \
    git  \
    nano \
    python3-pip 

RUN sudo mkdir AutoDogFeeder

# Just use rosdep i.e. include in package.xml. 
# For names, see (https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml)
# Instructions: (https://docs.ros.org/en/iron/How-To-Guides/Using-Python-Packages.html)
# It doesn't have gpiozero

# Install python modules
# COPY requirements.txt .

# You need to run this before to prevent the errors with pycairo
# (Optional)
# RUN sudo apt-get update && sudo apt-get upgrade -y

# (Required)
# RUN sudo apt-get install libcairo2-dev pkg-config python3-dev -y
# And finally
# RUN sudo python3 -m pip install --no-cache-dir -r requirements.txt


# I think I'll stick to mounts for now
# Copy project files (or build when creating image?)
# COPY build/ AutoDogFeeder/build
# COPY images/ AutoDogFeeder/images
# COPY install/ AutoDogFeeder/install
# COPY launch/ AutoDogFeeder/launch
# COPY log/ AutoDogFeeder/log
# COPY src/ AutoDogFeeder/src

# COPY run_demo.bash AutoDogFeeder/
# During development, the bind mount should overwrite these, at least according to https://stackoverflow.com/a/51358541

# Directory for storing videos
RUN sudo mkdir AutoDogFeeder/recordings

# Directory for fake camera input
COPY fake_video/ AutoDogFeeder/fake_video

# Rosdep update
RUN rosdep update

# Not sure if this has any effect on resolving the problem of 
# no communication between terminals(on vscode in Windows 10) 
# within the same container.
# might need this 
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# (Does not seem to help for now)

# Only when files are copied
# Install dependencies (this takes a while)
# RUN rosdep install -i --from-path AutoDogFeeder/src --rosdistro iron -y

# Can't run this since it fails for ros-iron-cv-bridge
# Need to manually run:
RUN sudo apt-get update
RUN sudo apt-get upgrade -y
RUN sudo apt-get install ros-iron-cv-bridge -y
RUN sudo apt-get install ros-iron-vision-opencv -y

# For a handful of modules, just do this. Consider poetry or something else if this grows.
# Also a venv if needed.
RUN sudo python3 -m pip install gpiozero RPLCD RPi.GPIO

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Build ros package
#RUN cd AutoDogFeeder
#RUN colcon build

# 
# ENTRYPOINT ["entrypoint.sh"]