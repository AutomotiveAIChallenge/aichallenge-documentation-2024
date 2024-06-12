# FAQ

## Environment Setup

### <u>The communication between AWSIM and Autoware is unstable.</u>

When testing locally, setting `ROS_LOCALHOST_ONLY=1` on all terminals improves communication speed. Add the following lines to your .bashrc:

```bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ ! -e /tmp/cycloneDDS_configured ]; then
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
fi
```

For this competition, we are considering configurations with two PCs: Windows+Linux or Linux+Linux.
In that case, set `ROS_LOCALHOST_ONLY=0`.

Note:

- After the OS starts, you will be prompted for a password when opening a terminal, and the initial `sudo ip link set lo multicast on` is required.
- If you forget to change the .bashrc as described above, it will always be applied, so be sure to check for changes with `echo $ROS_LOCALHOST_ONLY`.
- Communication between containers cannot occur if `ROS_LOCALHOST_ONLY=1` and `ROS_LOCALHOST_ONLY=0` are mixed.
- Note that `ROS_LOCALHOST_ONLY` is specified in the executable file.

---

### <u>ros2 topic list is not displayed.</u>

Make sure that the `ROS_DOMAIN_ID` of your machine matches. (There is no problem if you have not set the `ROS_DOMAIN_ID`.)
Also, please ensure that ROS2 is sourced.

---

### <u>Using AWSIM on Windows and Autoware on Ubuntu, the $ ros2 topic list is not displayed.</u>

Allow communication through the Windows Firewall.
Also, execute `ros2 daemon stop` and `ros2 daemon start` to check if any unnecessary processes remain, and restart.

---

### <u>Rocker does not start.</u>

First, check if rocker is installed.
If it is installed but does not start, check your permissions. There have been reports that it cannot be executed if the account type/permissions differ between the account that built the image and the account that runs it.

---

### <u>AWSIM terminates with a core dump.</u>

If AWSIM terminates with a core dump immediately after startup, the GPU memory may be insufficient. Therefore, check if the GPU memory usage has reached its limit with `nvidia-smi`.
It is recommended to have more than 11GB of GPU memory.

---

### <u>Only a Windows PC with a GPU is available.</u>

Since the support for this competition is based on the configuration described on the HP, detailed guidance cannot be provided, but the following methods are generally possible.

The problem is setting up the Autoware environment to participate.
Therefore, the key is to prepare an environment to run Autoware, and there may be issues with performance, package availability, and host-container communication settings, but the following methods are possible:

- Prepare Ubuntu with dual boot
- Prepare Ubuntu in a VM on Windows (Hyper-V, VirtualBox, VMware, etc.)
- Prepare Ubuntu on WSL2
- Prepare a docker environment on Windows (directly install the Autoware image)
- Build an environment in the cloud (some participants in past competitions have used AWS)

---

### <u>AWSIM is displayed after setting up the environment on AWS, but Rviz shows a black screen.</u>

There have been reports that `sudo apt upgrade` resolved the issue, so please check and try it.
Also, there was a similar question in a [past Issue](https://github.com/ros2/rviz/issues/948), so please check it as well.

---

## Operations

### ROS

#### <u>When creating a package in python, a "no module named \*" error occurs at runtime.</u>

Please refer to [this](https://zenn.dev/tasada038/articles/5d8ba66aa34b85#setup.py%E3%81%ABsubmodules%E3%81%A8%E3%81%97%E3%81%A6%E3%83%91%E3%83%83%E3%82%B1%E3%83%BC%E3%82%B8%E3%82%92%E8%BF%BD%E5%8A%A0%E3%81%99%E3%82%8B).

---

#### <u>How can I check the type of a topic?</u>

To check the type of a topic, use `ros2 topic info -v fuga_topic` or if you can identify the node, use `ros2 node info hoge-node`.
If you want to find more information about ROS, searching "ROS2 commands" on the internet might be helpful.

---

### Autoware

#### <u>Maps and routes are not displayed on Rviz.</u>

Make sure that the map data you are using is placed in the appropriate location and is correct.

---

#### <u>I don't know how to modify Autoware to participate.</u>

There are ways such as adjusting node parameters, modifying, or replacing nodes in Autoware.
The basic configuration of Autoware is summarized in another tab on this site and [here](https://automotiveaichallenge.github.io/aichallenge2023-integ/customize/index.html), so please make use of it.
Also, although it is an external article, [this](https://qiita.com/h_bog/items/86fba5b94b2148c4d9da) might be helpful.

---

#### <u>Tell me about behavior path/motion planner route generation.</u>

The behavior planner is primarily designed for driving on general roads (ODD3 and above), considering traffic rules that should not be broken (e.g., stop lines, crosswalks, signal stops).
Therefore, avoidance is also rule-based and not optimized.
On the other hand, motion is designed for driving in limited areas or limited spaces (ODD2 and below) and does not handle information such as signals or map information.
It is responsible for necessary functions such as obstacle avoidance, stopping, and speed optimization.

---

#### <u>Tell me about Autoware's avoidance behavior.</u>

There are two types of avoidance: behavior path and obstacle avoidance.
By default, obstacle avoidance is off, and only path smoothing is performed.
Additionally, by default, behavior path is set to avoid, but only cars and trucks are the target objects.

---

#### <u>Tell me about the center point.</u>

The center point detects vehicles, trucks, and pedestrians, but it cannot detect items like cardboard boxes that are not tagged.
However, currently, Autoware does not function unless it receives objects in the planning stage, and using the default configuration with the center point can cause the following issues:

1. Planning cannot generate a path if the center point fails.
2. Clustering-based obstacle detection results are erased by data association.

Therefore, the perception configuration of autoware mini is ideal, but understanding and implementing the addition, removal, and selection of nodes can be challenging, so it is important to ensure the center point functions correctly.
[Reference](https://autowarefoundation.github.io/autoware.universe/main/perception/lidar_centerpoint/)

---

### AWSIM

#### <u>How can I reset the car to its initial position?</u>

Currently, the only way is to restart AWSIM.

---

#### <u>AWSIM's operation is unstable.</u>

One of the causes can be insufficient GPU performance.
If using a high-performance GPU is difficult, setting the time scale to around 0.5 with the slider at the bottom of the AWSIM screen may stabilize the operation.

---

#### <u>I want to tune the mpc, but are the model parameters (lag and time constants) used in AWSIM disclosed?</u>

The lag and time constants are not measured or disclosed, but the basic specifications are available [here](https://automotiveaichallenge.github.io/aichallenge-documentation-2024/specifications/simulator/).

---

## General Competition

### <u>Is it possible to add additional sensors?</u>

To tackle the tasks under the same conditions and difficulty, adding new sensors is not allowed.

---
