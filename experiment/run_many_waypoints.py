#!/usr/bin/env python3
import numpy as np
import asyncio
import os
import sys
sys.path.append(os.getcwd()+'/experiment/src')
from ModelPredictiveControl import ModelPredictiveControl

if __name__ == '__main__':
    # dictionary for configuration
    # dt for Euler integration
    configDict = {"dt": 0.2, "stepNumHorizon": 10, "startPointMethod": "zeroInput"}
    config_file_name = 'experiment/config/config_dog.json'

    buildFlag = True
    saveFlag = True

    # x0 = np.array([-1.5, 0, 0])
    x0 = np.array([-2.046, 1.047, 0])
    u0 = np.array([0, 0, 0])
    T = 60
    # waypoints = [[0,0], [1,0]]
    # waypoints = [[-2.046, 1.047],
    #              [-1.734, 0.884],
    #              [-1.216, 0.644],
    #              [-0.882, 0.635],
    #              [-0.694, 0.474],
    #              [-0.639,-0.175],
    #              [-0.703,-0.357],
    #              [-1.202,-0.648],
    #              [-1.390,-0.633],
    #              [-1.549,-0.717],
    #              [-1.486,-0.826],
    #              [-1.228,-1.038],
    #              [-0.746,-1.050],
    #              [-0.228,-1.038],
    #              [-0.144,-0.886],
    #              [-0.137,-0.426],
    #              [ 0.002,-0.340],
    #              [ 0.479,-0.543],
    #              [ 0.796,-0.538],
    #              [ 1.500,-1.000]]
    waypoints = [[-1.734, 0.884],
                 [-1.216, 0.644],
                 [-0.882, 0.635],
                 [-0.694, 0.474],
                 [-0.639,-0.175],
                 [-0.703,-0.357],
                 [-1.202,-0.648],
                 [-1.390,-0.633],
                 [-1.549,-0.717],
                 [-1.486,-0.826],
                 [-1.228,-1.038],
                 [-0.746,-1.050],
                 [-0.228,-1.038],
                 [-0.144,-0.886],
                 [-0.137,-0.426],
                 [ 0.002,-0.340],
                 [ 0.479,-0.543],
                 [ 0.796,-0.538],
                 [ 1.500,-1.000]]

    # initialize MPC
    MyMPC = ModelPredictiveControl(configDict, buildFlag, waypoints, saveFlag, config_file_name)

    # Run our asynchronous main function forever
    asyncio.ensure_future(MyMPC.run(x0, T))
    asyncio.get_event_loop().run_forever()