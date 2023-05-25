# Installation

> Note: The installation process is a copy of [docker installation for development](../../../installation/autoware/docker-installation-devel/#docker-installation-for-development). Minor changes have been made for the F1TENTH demonstration.

## Prerequisites

- [Git](https://git-scm.com/)

- For NVIDIA Jetson devices, install [JetPack](https://docs.nvidia.com/jetson/jetpack/install-jetpack/index.html#how-to-install-jetpack) >= 5.0

## How to set up a development environment

1. Download & unzip simulator into home directory.

   ```bash
   cd ~
   wget https://github.com/amadeuszsz/AWSIM/releases/download/v1.1.0_f1tenth/AWSIM_v1.1.0_F1TENTH.zip
   unzip AWSIM_v1.1.0_F1TENTH.zip
   ```

2. Download & unzip map into home directory.

   ```bash
   cd ~
   wget https://github.com/amadeuszsz/AWSIM/releases/download/v1.1.0_f1tenth/autoware_map.zip
   unzip autoware_map.zip
   ```

3. Clone `PPI-PUT/autoware` into home directory.

  ```bash
  cd ~
  git clone https://github.com/PPI-PUT/autoware.git --branch f1tenth
  ```

> Note: Before installing NVIDIA libraries, confirm and agree with the licenses.

- [CUDA](https://docs.nvidia.com/cuda/eula/index.html)


### Installing dependencies manually

- [Install Nvidia CUDA](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/cuda#manual-installation)
- [Install Docker Engine](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/docker_engine#manual-installation)
- [Install NVIDIA Container Toolkit](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/nvidia_docker#manual-installation)
- [Install rocker](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rocker#manual-installation)

## How to set up a workspace

!!! warning

    Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license).
    By pulling and using the Autoware Universe images, you accept the terms and conditions of the license.

1. Pull the Docker image

   ```bash
   docker pull ghcr.io/autowarefoundation/autoware-universe:humble-latest-cuda-amd64
   ```

2. Launch a Docker container.

   - For amd64 architecture computers with NVIDIA GPU:

     ```bash
     cd ~/autoware
     ./run_amd64.sh
     ```

   - If you want to run container without using NVIDIA GPU, or for arm64 architecture computers:

     ```bash
     ./run_arm64.sh
     ```

    !!! warning
    
        Arm64 architecture computers require a built Rocker from sources or v0.2.13+ release.

   For more advanced usage, see [here](https://github.com/autowarefoundation/autoware/tree/main/docker/README.md).

   After that, move to the workspace in the container:

   ```bash
   cd autoware
   ```

3. Create the `src` directory and clone repositories into it.

   ```bash
   mkdir src
   vcs import src < autoware.repos
   vcs import src < f1tenth.repos
   ```

4. Update dependent ROS packages.

   The dependency of Autoware may change after the Docker image was created.
   In that case, you need to run the following commands to update the dependency.

   ```bash
   sudo apt update
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

5. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1
   ```

   If there is any build issue, refer to [Troubleshooting](../../support/troubleshooting/index.md#build-issues).

6. To enter the container with new terminal window, run the following command:
   
   ```bash
   cd ~/autoware
   ./enter.sh
   ```

## How to update a workspace

1. Update the Docker image.

   ```bash
   docker pull ghcr.io/autowarefoundation/autoware-universe:humble-latest-cuda-amd64
   ```

2. Launch a Docker container.

   - For amd64 architecture computers:

     ```bash
     cd ~/autoware
     ./run_amd64.sh
     ```

   - If you want to run container for arm64 architecture computers.

     ```bash
     cd ~/autoware
     ./run_arm64.sh
     ```

3. Update the `.repos` file.

   ```bash
   cd autoware
   git pull
   ```

4. Update the repositories.

   ```bash
   vcs import src < autoware.repos
   vcs import src < f1tenth.repos
   vcs pull src
   ```

5. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1
   ```
