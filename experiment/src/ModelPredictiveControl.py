#!/usr/bin/env python3
import time
import copy
import numpy as np
import math
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.getcwd()+'/src')
sys.path.append('externals/unitree_legged_sdk/lib/python/amd64')
from DogSys import DogSys
from OptimalControl import OptimalControl
import json
import transforms3d
import csv

import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import qtm

import math
print(os.getcwd())
import robot_interface as sdk

class ModelPredictiveControl:
    configDict: dict  # a dictionary for parameters

    def __init__(self, configDict: dict, buildFlag=True, waypoints=[[0,0]], saveFlag=False, config_file_name='experiment/config/config_dog.json'):
        self.configDict = configDict
        self.dt = self.configDict['dt']
        self.saveFlag = saveFlag

        # initialize DogSys
        self.MyDogSys = DogSys(configDict, buildFlag)
        self.waypoints = waypoints
        self.offset = 0.1
        # initialize OptimalControlProblem
        self.MyDogOc = OptimalControl(configDict, self.MyDogSys, buildFlag)
        
        # method
        try:
            self.methodStr = self.configDict["method"]
        # proposed MPC scheme by default
        except:
            print("No setting for method. Set MPC by default")
            self.methodStr = "MPC"

        
        self.numWaypoints = len(self.waypoints)

        # Read the configuration from the json file
        self.json_file = open(config_file_name)
        self.config_data = json.load(self.json_file)
        self.IP_server = self.config_data["QUALISYS"]["IP_MOCAP_SERVER"]

        # Initialize dog
        HIGHLEVEL = 0xee
        LOWLEVEL  = 0xff

        self.udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)

        self.cmd = sdk.HighCmd()
        state = sdk.HighState()
        self.udp.InitCmdData(self.cmd)

    def create_body_index(self, xml_string):
        """ Extract a name to index dictionary from 6-DOF settings xml """
        xml = ET.fromstring(xml_string)

        body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_to_index[body.text.strip()] = index

        return body_to_index

    def publish_vel(self):
        self.pub_move.publish(self.move)

    async def run(self, iniState: np.array, timeTotal: float):
        # Connect to qtm
        connection = await qtm.connect(self.IP_server)

        # Connection failed?
        if connection is None:
            print("Failed to connect")
            return

        # Take control of qtm, context manager will automatically release control after scope end
        async with qtm.TakeControl(connection, "password"):
            pass

        # Get 6-DOF settings from QTM
        xml_string = await connection.get_parameters(parameters=["6d"])

        # parser for mocap rigid bodies indexing
        body_index = self.create_body_index(xml_string)
        # the one we want to access
        wanted_body = self.config_data["QUALISYS"]["NAME_SINGLE_BODY"]
        
        self.time_name = time.strftime("%Y%m%d%H%M%S")
        self.t_start = time.time()
        self.timeNow = time.time() - self.t_start  # [sec]
        self.stateNow = copy.deepcopy(iniState)
        self.idx = 0
        self.waypoint = self.waypoints[0]
        self.reached = 0

        # initialize trajectories for states, inputs, etc.
        self.timeTraj = np.array([self.timeNow], dtype=np.float64)
        # trajectory for states
        self.xTraj = np.zeros((1, self.MyDogSys.dimStates), dtype=np.float64)
        self.xTraj[0, :] = self.stateNow
        # trajectory for input
        self.uTraj = np.zeros((1, self.MyDogSys.dimInputs), dtype=np.float64)
        # trajectory for entire optimization time [sec]
        self.algTimeTraj = np.zeros(1, dtype=np.float64)
        # trajectory for Ipopt solution time [sec]
        self.ipoptTimeTraj = np.zeros(1, dtype=np.float64)
        # list for logging ipopt status
        self.logTimeTraj = list()
        self.logStrTraj = list()

        # load function to run optimization once
        if self.methodStr == "MPC":
            self.runOnce = lambda stateNow, timeNow, waypoint: self._runOC(self.stateNow, self.timeNow, self.waypoint)
        else:
            raise Exception("Wrong method in configDict!")

        self.reached = 0

        def on_packet(packet):
            # dog setting
            self.cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
            self.cmd.gaitType = 0
            self.cmd.speedLevel = 0
            self.cmd.footRaiseHeight = 0
            self.cmd.bodyHeight = 0
            self.cmd.euler = [0, 0, 0]
            self.cmd.velocity = [0, 0]
            self.cmd.yawSpeed = 0.0
            self.cmd.reserve = 0

            # Get the 6-DOF data
            bodies = packet.get_6d()[1]
            # Extract one specific body
            wanted_index = body_index[wanted_body]
            position, rotation = bodies[wanted_index]
            # You can use position and rotation here. Notice that the unit for position is mm!
            # print(wanted_body)

            # rotation.matrix is a tuple with 9 elements.
            rotation_np = np.asarray(rotation.matrix, dtype=float).reshape(3, 3)

            # send 6-DOF data via TCP
            # concatenate the position and rotation matrix vertically
            # msg = np.asarray((position.x/1000.0, position.y/1000.0, position.z/1000.0) + rotation.matrix, dtype=float).tobytes()

            quat = transforms3d.quaternions.mat2quat(rotation_np)
            # print("quat")
            # print(quat)

            data = np.array([position.x/1000.0, position.y/1000.0, position.z/1000.0, quat[0], quat[1], quat[2], quat[3], self.timeNow], dtype=float)

            # # for debugging
            # print("rotation matrix in array")
            # print(rotation.matrix)
            # print("rotation matrix in matrix")
            # print(rotation_np)

            roll_now, pitch_now, yaw_now = transforms3d.euler.quat2euler(quat, axes='sxyz')
            yaw_now = -yaw_now

            xy = [position.x/1000.0, position.y/1000.0]
            
            # print(position)
            # print("yaw")
            # print(yaw_now)
            # print(math.degrees(yaw_now))
            # print((xy[0] - self.waypoint[0])**2 + (xy[1] - self.waypoint[1])**2)
            if (self.reached < self.numWaypoints):               
                # solve
                ipoptTime, returnStatus, successFlag, algoTime, print_str, uNow = self.runOnce(self.stateNow, self.timeNow, self.waypoint)

                # apply control and forward propagate dynamics
                # xNext = self.MyDogSys.discDynFun(self.stateNow, uNow)
                xNext = [xy[0], xy[1], yaw_now]

                # update trajectory
                self.stateNow = np.array(xNext).reshape((1,-1)).flatten()
                self.xTraj = np.vstack((self.xTraj, self.stateNow))
                if self.idx <= 0.5:
                    self.uTraj[self.idx, :] = uNow
                    self.algTimeTraj[self.idx] = algoTime
                    self.ipoptTimeTraj[self.idx] = ipoptTime
                else:
                    self.uTraj = np.vstack((self.uTraj, uNow))
                    self.algTimeTraj = np.append(self.algTimeTraj, algoTime)
                    self.ipoptTimeTraj = np.append(self.ipoptTimeTraj, ipoptTime)

                if self.idx % 50 == 0:
                    # print(print_str)
                    pass
                if not successFlag:
                    # if self.idx % 50 != 0:
                        # print(print_str)
                    # print(returnStatus)
                    self.logTimeTraj.append(self.timeNow)
                    self.logStrTraj.append(returnStatus)

                # update time
                self.timeNow = time.time() - self.t_start
                self.idx += 1
                self.timeTraj = np.append(self.timeTraj, self.timeNow)


                if ((self.stateNow[0]-self.waypoints[self.reached][0])**2+(self.stateNow[1]-self.waypoints[self.reached][1])**2 <= self.offset**2):
                    self.reached += 1
                    print("reached waypoint")
                    print(self.reached)
                    print(self.waypoint)
                    if self.reached < self.numWaypoints:
                        self.waypoint = self.waypoints[self.reached]

                # print(print_str)
                        
                # run high level
                # print(xy)
                # print(uNow)
                self.cmd.mode = 2
                self.cmd.gaitType = 1
                self.cmd.velocity = [uNow[0], uNow[1]]
                self.cmd.yawSpeed = uNow[2]
                self.cmd.bodyHeight = 0

                self.udp.SetSend(self.cmd)
                self.udp.Send()
                
                # time.sleep(self.dt)  
            
            else:
                # Finish up
                result = {"timeTraj": self.timeTraj,
                            "xTraj": self.xTraj,
                            "uTraj": self.uTraj,
                            "ipoptTimeTraj": self.ipoptTimeTraj,
                            "logTimeTraj": self.logTimeTraj,
                            "logStrTraj": self.logStrTraj}

                if self.saveFlag:
                    with open('experiment/traj/' + self.time_name + '.csv', 'w') as file:
                        writer = csv.writer(file)
                        print(self.timeTraj)
                        writer.writerow(self.timeTraj)
                        print(self.MyDogSys.dimStates)
                        for idx in range(self.MyDogSys.dimStates):
                            writer.writerow(self.xTraj.T[idx])
                        for idx in range(self.MyDogSys.dimInputs):
                            writer.writerow(self.uTraj.T[idx])

                raise Exception("Done")

        # Start streaming frames
        # Make sure the component matches with the data fetch function, for example: packet.get_6d() with "6d"
        # Reference: https://qualisys.github.io/qualisys_python_sdk/index.html
        await connection.stream_frames(components=["6d"], on_packet=on_packet)

    def _runOC(self, stateNow, timeNow, waypoint):
        t0 = time.time()
        xTrajNow, uTrajNow, timeTrajNow, ipoptTime, returnStatus, successFlag = self.MyDogOc.solve(stateNow, timeNow, waypoint)
        t1 = time.time()
        algoTime = t1 - t0
        print_str = "Sim time [sec]: " + str(round(timeNow, 1)) + "   Comp. time [sec]: " + str(round(algoTime, 3))
        # apply control
        uNow = uTrajNow[0, :]
        return ipoptTime, returnStatus, successFlag, algoTime, print_str, uNow


    def visualize(self, result: dict, matFlag=False, legendFlag=True, titleFlag=True, blockFlag=True):
        """
        If result is loaded from a .mat file, matFlag = True
        If result is from a seld-defined dict variable, matFlag = False
        """
        timeTraj = result["timeTraj"]
        xTraj = result["xTraj"]
        uTraj = result["uTraj"]

        if matFlag:
            timeTraj = timeTraj[0, 0].flatten()
            xTraj = xTraj[0, 0]
            uTraj = uTraj[0, 0]

        # trajectories for states
        fig1, ax1 = plt.subplots(2, 1)
        if titleFlag:
            fig1.suptitle("State Trajectory")
        ax1[0].plot(xTraj[:,0], xTraj[:,1], color="blue", linewidth=2)

        ax1[0].set_xlabel("x [m]")
        ax1[0].set_ylabel("y [m]")

        ax1[1].plot(timeTraj, xTraj[:,0], color="blue", linewidth=2)
        ax1[1].plot(timeTraj, xTraj[:,1], color="red", linewidth=2)
        ax1[1].set_xlabel("time [sec]")
        ax1[1].set_ylabel("x,y [m]")

        # trajectories for inputs
        fig2, ax2 = plt.subplots(2, 1)
        if titleFlag:
            fig2.suptitle("Input Trajectory")
        # trajectory for current
        ax2[0].plot(timeTraj[:-1], uTraj[:,0], color="blue", linewidth=2)
        # for input bounds
        ax2[0].plot(timeTraj[:-1],
            self.MyDogOc.lin_vel_lb*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        ax2[0].plot(timeTraj[:-1],
            self.MyDogOc.lin_vel_ub*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        # ax2[0].set_xlabel("time [sec]")
        ax2[0].set_ylabel(r'$v \  [\mathrm{m/s}]$')

        ax2[1].plot(timeTraj[:-1], uTraj[:,1], color="blue", linewidth=2)
        # for input bounds
        ax2[1].plot(timeTraj[:-1],
            self.MyDogOc.ang_vel_lb*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        ax2[1].plot(timeTraj[:-1],
            self.MyDogOc.ang_vel_ub*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        ax2[1].set_xlabel("time [sec]")
        ax2[1].set_ylabel(r'$w\  [\mathrm{rad/s}]$')
        plt.tight_layout()
        plt.show(block=blockFlag)


if __name__ == '__main__':
    # dictionary for configuration
    # dt for Euler integration
    configDict = {"dt": 0.1, "stepNumHorizon": 5, "startPointMethod": "zeroInput"}
    config_file_name = 'experiment/config/config_dog.json'

    buildFlag = True
    saveFlag = False

    

    x0 = np.array([0, 0, 0])
    u0 = np.array([0, 0, 0])
    T = 20
    waypoints = [[-0.,0.], [1,-0.5], [2,0]]

    # initialize MPC
    MyMPC = ModelPredictiveControl(configDict, buildFlag, waypoints, saveFlag, config_file_name)

    # Run our asynchronous main function forever
    asyncio.ensure_future(MyMPC.run(x0, T))
    asyncio.get_event_loop().run_forever()

