# Robotic-Dog-Tracking-Interface
This GitHub repo is a tutorial and a waypoint/trajectory tracking interface for [Unitree Go1 Robot](https://shop.unitree.com/), please refer to official [sdk](https://github.com/unitreerobotics/unitree_legged_sdk/tree/go1) for details. 
* The tutorial includes the setup of the robot, network connections, low-level-control and high-level-control examples.
* Robotic-Dog-Tracking-Interface is able to obtain state estimation from motion capture systems [Qualisys](https://www.qualisys.com/), and you're welcome to write functions for your own motion capture system. The purpose of this repository is that prople can foucs on motion planning / path planning / trajectory planning and **Robotic-Dog--Tracking-Interface** can take over actual waypoint/trajectory tracking as long as your planner outputs waypoints/trajectory as csv files in a specific directory.

**Future updates**
* Direct low-level-control examples
* Trajectory tracking

Dependencies
============
This repo has been tested with:
* Ubuntu 20.04 LTS, Python 3.8
* Raspberry Pi

Requuired packages:
```
$ sudo apt install cmake libmsgpack* libboost-all-dev
```

Build
=====
To download and build this repo,
```
$ git clone https://github.com/tianyuzhou-sam/Robotic-Dog-Tracking-Interface.git
$ cd <MAIN_DIRECTORY>
$ git submodule update --init --recursive
$ cd <MAIN_DIRECTORY>
$ mkdir build
$ cd build
$ cmake ..
$ make
```

If you want to build the python wrapper, then replace the cmake line `cmake ..` with:
```
$ cmake -DPYTHON_BUILD=TRUE ..
```

Connection
==========
**1. To run on the robot's system (raspberry pi):** 

First connect your computer to the robot's wifi `Unitree_Go429210A` (5 GHz only) with password `00000000`. Then open a new terminal
```
$ ssh <USERNAME>@192.168.12.1
```
```
$ ssh pi@192.168.12.1
```
Enter the password and then you are in the robot's system. The default password is `123`.


**2. To run on network device, a bridge is needed:**
On the robot's system (in the same terminal that you just ssh-ed):
```
sudo sysctl -p
sudo iptables -F
sudo iptables -t nat -F
sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i wlan1 -o eth0 -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan1 -j ACCEPT

```
On network device (Open a new terminal in your computer):
```
$ sudo route add default gw 192.168.12.1
```


Example
=======

The same computer needs to connect with the motion capture system with Ethernet cables.

To run an open-loop walking example:
```
$ cd build
$ ./example_walk
```


To run NMPC waypoint tracking with the motion capture system:

```
$ python3 experiment/run_waypoints.py
```
