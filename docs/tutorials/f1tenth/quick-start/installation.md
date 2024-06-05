# Installation

Here we describe a process of setting up F1TENTH environment for desktops and real vehicles. For now, we don't support docker installation on Jetson devices due to camera driver issues. Follow the instruction for your target platform.

!!! note

    The installation process is a copy of [docker installation for development](../../../../installation/autoware/docker-installation-devel/). Changes have been made for F1TENTH purposes.

## Prerequisites

### Desktop

- [Install Nvidia CUDA](https://github.com/autowarefoundation/autoware/tree/2024.02/ansible/roles/cuda#manual-installation)
- [Install Docker Engine](https://github.com/autowarefoundation/autoware/tree/2024.02/ansible/roles/docker_engine#manual-installation)
- [Install NVIDIA Container Toolkit](https://github.com/autowarefoundation/autoware/tree/2024.02/ansible/roles/nvidia_docker#manual-installation)
- [Install rocker](https://github.com/autowarefoundation/autoware/tree/2024.02/ansible/roles/rocker#manual-installation)

### Nvidia Jetson

- [JetPack](https://developer.nvidia.com/embedded/jetpack) >= 6.0

## Configure system

### Desktop & Jetson

#### Enable multicasting

1. Create the following file: `sudo touch /etc/systemd/system/multicasting.service`:

   ```bash
   [Unit]
   Description=Setting up local network multicasting
   [Service]
   ExecStart=/bin/bash -c 'ip link set lo multicast on'
   Restart=no
   [Install]
   WantedBy=multi-user.target
   ```

2. Enable the service:

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable multicasting.service
   ```

#### Configure kernel buffer

1. Create the following file: `sudo touch /etc/sysctl.d/10-cyclone-max.conf`:

   ```bash
   net.core.rmem_max=2147483647
   net.ipv4.ipfrag_time=3
   net.ipv4.ipfrag_high_thresh=134217728 # (128 MB)
   ```

2. Either restart the computer or run following to enable the changes:

   ```bash
   sudo sysctl -w net.core.rmem_max=2147483647
   sudo sysctl -w net.ipv4.ipfrag_time=3
   sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
   ```

#### Configure shell prompt

1. Add these lines to `~/.bashrc` to see git repo’s info and extra line prefix. It will make you sure that currently your active terminal tab is on host.

   ```bash
   export GIT_PS1_SHOWDIRTYSTATE=1
   export GIT_PS1_SHOWSTASHSTATE=1
   export GIT_PS1_SHOWUNTRACKEDFILES=1
   export GIT_PS1_SHOWUPSTREAM="verbose"
   export GIT_PS1_DESCRIBE_STYLE=contains
   export GIT_PS1_SHOWCOLORHINTS=1
   PRE='\[\033[01;31m\](host) \[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]'
   export PROMPT_COMMAND="__git_ps1 '"'${VIRTUAL_ENV:+($(basename "$VIRTUAL_ENV")) }'"$PRE' '$ '"
   ```

2. For better experience install terminator terminal: `sudo apt install terminator`.

### Jetson only

1. Add swap:

   ```bash
   sudo fallocate -l 32G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

2. Add these lines to `~/.bashrc`:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/$USER/autoware/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_LOCALHOST_ONLY=1
   export ROS_DOMAIN_ID=32
   export RCUTILS_COLORIZED_OUTPUT=1
   export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
   export CYCLONEDDS_URI=file:///home/$USER/cyclonedds_config.xml
   ```

3. Load custom kernel modules if needed
   
   * Download joystick kernel module [joydev.ko](https://drive.google.com/file/d/1KFrF6slMr7cHgh-mxbgcM6Rd9sgfR2Ju)

      ```bash
      cd path/to/file
      sudo mv joydev.ko /lib/modules/$(uname -r)/kernel/drivers/input/
      sudo modprobe joydev
      ```

   * Download Xsens AHRS kernel module [xsens_mt.ko](https://drive.google.com/file/d/17uZfMHGgip2ZGZs79n33--3321VYAsqL)

      ```bash
      cd path/to/file
      sudo mv xsens_mt.ko /lib/modules/$(uname -r)/kernel/drivers/input/
      sudo modprobe xsens_mt
      ```

## How to install

Desktop and Jetson installation process differ in two aspects - desktops platforms need Docker and of course, AWSIM simulator. Even though you don't need containerization for running F1TENTH on desktop platforms, we strongly recommend to use our Docker image.

### Desktop

1. Clone meta repository:
  ```bash
  cd ~
  git clone https://github.com/amadeuszsz/autoware.git --branch f1tenth
  ```

2. Pull base docker image & build F1TENTH Docker image:

   ```bash
   docker pull ghcr.io/autowarefoundation/autoware-universe:humble-2024.02-cuda
   cd ~/autoware/.devcontainer
   ./build.sh
   ```

3. Download & unzip simulator into home directory:

   ```bash
   cd ~
   wget https://github.com/amadeuszsz/AWSIM/releases/download/v1.2.0_f1tenth/AWSIM_v1.2.0_F1TENTH.zip
   unzip AWSIM_v1.2.0_F1TENTH.zip -d autoware_awsim
   ```

4. Download & unzip map into home directory:

   ```bash
   cd ~
   wget https://github.com/amadeuszsz/AWSIM/releases/download/v1.2.0_f1tenth/race_track_01.zip
   unzip race_track_01.zip -d autoware_map
   ```

5. Create data directory:

   ```bash
   cd ~
   mkdir autoware_data
   ```

6. Run Docker container & build workspace:

   ```bash
   cd ~/autoware
   ./run.sh
   cd ~/autoware
   mkdir src
   vcs import src < autoware.repos
   vcs import src < f1tenth_awsim.repos
   sudo apt update
   rosdep update
   rosdep install -yr --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On
   ```

7. You can enter to the container in new terminal window using provided script:

   ```bash
   cd ~/autoware
   ./enter.sh
   ```

!!! note

    From now on, all the commands for the desktops have to be performed in Docker container.

### Jetson

1. Install ROS 2 with official [instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2. Clone meta repository:
  ```bash
  cd ~
  git clone https://github.com/amadeuszsz/autoware.git --branch f1tenth
  ```

1. Create map directory:

   ```bash
   cd ~
   mkdir autoware_map
   ```

2. Create data directory:

   ```bash
   cd ~
   mkdir autoware_data
   ```

3. Build workspace:

   ```bash
   cd ~/autoware
   mkdir src
   vcs import src < autoware.repos
   vcs import src < f1tenth_awsim.repos
   vcs import src < f1tenth_xray.repos
   sudo apt update
   rosdep update
   rosdep install -yr --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --continue-on-error
   ```

## How to update a workspace

1. Update meta repository.

   ```bash
   cd ~/autoware
   git pull
   ```

2. Update the repositories.

   ```bash
   vcs import src < autoware.repos
   vcs import src < f1tenth_awsim.repos
   vcs import src < f1tenth_xray.repos  # only for Jetson
   vcs pull src
   ```

3. Build the workspace.

   ```bash
   sudo apt update
   rosdep install -yr --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --continue-on-error
   ```
