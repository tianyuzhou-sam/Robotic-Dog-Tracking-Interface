#!/usr/bin/env python3
import os
import sys
import numpy as np
sys.path.append(os.getcwd()+'/src')
from ModelPredictiveControl import ModelPredictiveControl

if __name__ == '__main__':
    # dictionary for configuration
    # dt for Euler integration
    configDict = {"dt": 0.1, "stepNumHorizon": 10, "startPointMethod": "zeroInput"}

    buildFlag = True
    saveFlag = False

    x0 = np.array([0, 0, 0])
    u0 = np.array([0, 0])
    targets = [[5,5], [10,5], [10,10]]
    T = 100

    # initialize MPC
    MyMPC = ModelPredictiveControl(configDict, buildFlag, targets, saveFlag)
    result = MyMPC.run(x0, T)
    print(result)
    MyMPC.visualize(result)