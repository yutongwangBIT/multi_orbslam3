# Multi ORB-SLAM3

multi_orbslam3 is a repository that integrates the collaborative method of CCM-SLAM into the ORB-SLAM3 framework.

## Introduction

ORB-SLAM3 is a versatile and accurate SLAM solution for monocular, stereo and RGB-D cameras. It can compute the camera trajectory and a sparse 3D reconstruction in real-time on standard CPUs.

CCM-SLAM is a distributed SLAM system that enables multiple agents to collaboratively explore and map an unknown environment in real-time.

Multi ORB-SLAM3 combines the advantages of both systems and allows multiple cameras to perform collaborative SLAM using the ORB-SLAM3 framework.

## Installation

The installation of multi_orbslam3 requires the same libraries as ORB-SLAM3. Please follow the instructions on [this page](https://github.com/UZ-SLAMLab/ORB_SLAM3#1-prerequisites) to install them.

**Note: This repository has been tested on Ubuntu 18.04 and ROS Melodic. Other versions may not be compatible.**

To build multi_orbslam3, you need to clone this repository and run the following commands:

```bash
cd multi_orbslam3 
catkin build

```

**Note: We use catkin build instead of catkin_make.**

## Usage

To run Multi ORB-SLAM3, you need to launch one server node and multiple client nodes. Each client node corresponds to one camera that performs local SLAM and sends its data to the server node. The server node performs global optimization and fusion of the data from different clients.

You can use the provided scripts to launch the nodes:

```
./launch_server.sh 
./launch_client_1.sh 
./launch_client_2.sh
...
```

You can also modify the scripts to change the parameters or input sources of each node.

## Contribution

We welcome any contributions or suggestions for improving Multi ORB-SLAM3. You can open an issue or submit a pull request on GitHub.

## License

Multi ORB-SLAM3 is released under the GNU General Public License v3.0. Please see [LICENSE](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/License-gpl.txt) for more details.
