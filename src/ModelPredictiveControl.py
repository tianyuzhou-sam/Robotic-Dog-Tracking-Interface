#!/usr/bin/env python3
import time
import copy
import numpy as np
import matplotlib.pyplot as plt
import csv
from DogSys import DogSys
from OptimalControl import OptimalControl


class ModelPredictiveControl:
    configDict: dict  # a dictionary for parameters

    def __init__(self, configDict: dict, buildFlag=True, targets=[[0,0]], saveFlag=False):
        self.configDict = configDict
        self.saveFlag = saveFlag

        # initialize DogSys
        self.MyDogSys = DogSys(configDict, buildFlag)
        self.targets = targets

        # initialize OptimalControlProblem
        self.MyOC = OptimalControl(configDict, self.MyDogSys, buildFlag)
        
        # method
        try:
            self.methodStr = self.configDict["method"]
        # proposed MPC scheme by default
        except:
            print("No setting for method. Set MPC by default")
            self.methodStr = "MPC"
  
        self.numTargets = len(self.targets)
        self.offset = 0.1


    def run(self, iniState: np.array, timeTotal: float):
        timeNow = 0.0  # [sec]
        stateNow = copy.deepcopy(iniState)
        idx = 0
        target = self.targets[0]

        # initialize trajectories for states, inputs, etc.
        timeTraj = np.array([timeNow], dtype=np.float64)
        # trajectory for states
        xTraj = np.zeros((1, self.MyDogSys.dimStates), dtype=np.float64)
        xTraj[0, :] = stateNow
        # trajectory for input
        uTraj = np.zeros((1, self.MyDogSys.dimInputs), dtype=np.float64)
        # trajectory for entire optimization time [sec]
        algTimeTraj = np.zeros(1, dtype=np.float64)
        # trajectory for Ipopt solution time [sec]
        ipoptTimeTraj = np.zeros(1, dtype=np.float64)
        # list for logging ipopt status
        logTimeTraj = list()
        logStrTraj = list()

        # target = self.targets[0]
        # self.MyOC = OptimalControlProblem(configDict, self.MyDogSys, buildFlag, target)
        # load function to run optimization once
        if self.methodStr == "MPC":
            self.runOnce = lambda stateNow, timeNow, target: self._runOC(stateNow, timeNow, target)
        else:
            raise Exception("Wrong method in configDict!")

        reached = 0

        while (timeNow <= timeTotal-self.MyOC.dt):

            # solve
            ipoptTime, returnStatus, successFlag, algoTime, print_str, uNow = self.runOnce(stateNow, timeNow, target)

            # apply control and forward propagate dynamics
            xNext = self.MyDogSys.discDynFun(stateNow, uNow)

            # update trajectory
            stateNow = np.array(xNext).reshape((1,-1)).flatten()
            xTraj = np.vstack((xTraj, stateNow))
            if idx <= 0.5:
                uTraj[idx, :] = uNow
                algTimeTraj[idx] = algoTime
                ipoptTimeTraj[idx] = ipoptTime
            else:
                uTraj = np.vstack((uTraj, uNow))
                algTimeTraj = np.append(algTimeTraj, algoTime)
                ipoptTimeTraj = np.append(ipoptTimeTraj, ipoptTime)

            if idx % 50 == 0:
                print(print_str)
            if not successFlag:
                if idx % 50 != 0:
                    print(print_str)
                print(returnStatus)
                logTimeTraj.append(timeNow)
                logStrTraj.append(returnStatus)

            # update time
            timeNow += self.MyOC.dt
            idx += 1
            timeTraj = np.append(timeTraj, timeNow)

            if (reached < self.numTargets):
                if ((stateNow[0] - self.targets[reached][0])**2 + (stateNow[1] - self.targets[reached][1])**2 <= self.offset**2):
                    reached += 1
                    if reached < self.numTargets:
                        target = self.targets[reached]

        print(print_str)
        result = {"timeTraj": timeTraj,
                  "xTraj": xTraj,
                  "uTraj": uTraj,
                  "ipoptTimeTraj": ipoptTimeTraj,
                  "logTimeTraj": logTimeTraj,
                  "logStrTraj": logStrTraj}

        if self.saveFlag:
            with open('controlData.csv', 'w') as file:
                writer = csv.writer(file)
                writer.writerow(timeTraj)
                for idx in range(self.MyDogSys.dimStates):
                    writer.writerow(xTraj.T[idx])
                for idx in range(self.MyDogSys.dimInputs):
                    writer.writerow(uTraj.T[idx])

        return result

    def _runOC(self, stateNow, timeNow, target):
        t0 = time.time()
        xTrajNow, uTrajNow, timeTrajNow, ipoptTime, returnStatus, successFlag = self.MyOC.solve(stateNow, timeNow, target)
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
        fig1, ax1 = plt.subplots()
        if titleFlag:
            fig1.suptitle("State Trajectory")
        ax1.plot(xTraj[:,0], xTraj[:,1], color="blue", linewidth=2)
        for idx in range(len(self.targets)):
            ax1.plot(self.targets[idx][0], self.targets[idx][1], 'kX')
        ax1.plot(self.targets[-1][0], self.targets[-1][1], 'r*', markersize=10)
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")

        # trajectories for inputs
        fig2, ax2 = plt.subplots(3, 1)
        if titleFlag:
            fig2.suptitle("Input Trajectory")
        # trajectory for current
        ax2[0].plot(timeTraj[:-1], uTraj[:,0], color="blue", linewidth=2)
        # for input bounds
        ax2[0].plot(timeTraj[:-1],
            self.MyOC.lin_vel_x_lb*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        ax2[0].plot(timeTraj[:-1],
            self.MyOC.lin_vel_x_ub*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        # ax2[0].set_xlabel("time [sec]")
        ax2[0].set_ylabel(r'$v \  [\mathrm{m/s}]$')

        ax2[1].plot(timeTraj[:-1], uTraj[:,1], color="blue", linewidth=2)
        # for input bounds
        ax2[1].plot(timeTraj[:-1],
            self.MyOC.lin_vel_y_lb*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        ax2[1].plot(timeTraj[:-1],
            self.MyOC.lin_vel_y_ub*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        # ax2[0].set_xlabel("time [sec]")
        ax2[1].set_ylabel(r'$v \  [\mathrm{m/s}]$')

        ax2[2].plot(timeTraj[:-1], uTraj[:,2], color="blue", linewidth=2)
        # for input bounds
        ax2[2].plot(timeTraj[:-1],
            self.MyOC.ang_vel_lb*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        ax2[2].plot(timeTraj[:-1],
            self.MyOC.ang_vel_ub*np.ones(timeTraj[:-1].size),
            color="black", linewidth=2, linestyle="dashed")
        ax2[2].set_xlabel("time [sec]")
        ax2[2].set_ylabel(r'$w\  [\mathrm{rad/s}]$')
        plt.tight_layout()
        plt.show(block=blockFlag)


if __name__ == '__main__':
    # dictionary for configuration
    # dt for Euler integration
    configDict = {"dt": 0.1, "stepNumHorizon": 10, "startPointMethod": "zeroInput"}

    buildFlag = True
    saveFlag = False

    x0 = np.array([0, 0, 0])
    u0 = np.array([0, 0, 0])
    targets = [[2,2], [3,2], [3,3]]
    T = 20

    # initialize MPC
    MyMPC = ModelPredictiveControl(configDict, buildFlag, targets, saveFlag)
    result = MyMPC.run(x0, T)
    print(result)
    MyMPC.visualize(result)

    # with open('controlData.csv') as csv_file:
    #     reader = csv.reader(csv_file)
    #     idx = 0
    #     for row in reader:
    #         if idx == 0:
    #             timeTraj = row
    #         elif idx <= 3:
    #             if idx == 1:
    #                 xTraj = row
    #             else:
    #                 xTraj = np.vstack((xTraj, row))
    #         else:
    #             if idx == 4:
    #                 uTraj = row
    #             else:
    #                 uTraj = np.vstack((uTraj, row))
    #         idx += 1

    # print(xTraj[0])

    # # trajectories for states
    # fig1, ax1 = plt.subplots()
    # fig1.suptitle("Trajectory")
    # ax1.plot(xTraj[0], xTraj[1], color="blue", linewidth=2)
    # ax1.set_xlabel("x [m]")
    # ax1.set_ylabel("y [m]")
    # plt.show()