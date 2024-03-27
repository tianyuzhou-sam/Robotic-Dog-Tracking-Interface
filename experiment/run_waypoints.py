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

    x0 = np.array([-1.5, 0, 0])
    u0 = np.array([0, 0, 0])
    T = 60
    waypoints = [[0,0], [1,0]]

    # initialize MPC
    MyMPC = ModelPredictiveControl(configDict, buildFlag, waypoints, saveFlag, config_file_name)

    # Run our asynchronous main function forever
    asyncio.ensure_future(MyMPC.run(x0, T))
    asyncio.get_event_loop().run_forever()
