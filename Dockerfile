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
    python3-pip \


# Install python modules
COPY requirements.txt ./
RUN sudo pip install --no-cache-dir -r requirements.txt

# Copy project files (or build when creating image?)
RUN sudo mkdir AutoDogFeeder
#COPY build/ AutoDogFeeder/build
COPY images/ AutoDogFeeder/images
#COPY install/ AutoDogFeeder/install
COPY launch/ AutoDogFeeder/launch
#COPY log/ AutoDogFeeder/log
COPY src/ AutoDogFeeder/src
COPY run_demo.bash AutoDogFeeder/
# During development, the bind mount should overrite these, at least according to https://stackoverflow.com/a/51358541

# Directory for storing videos
RUN sudo mkdir AutoDogFeeder/recordings

# Rosdep update
RUN rosdep update

# Install dependencies (this takes a while)
RUN rosdep install -i --from-path src --rosdistro iron -y

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Build ros package
RUN cd AutoDogFeeder && colcon build

# 
# ENTRYPOINT ["entrypoint.sh"]