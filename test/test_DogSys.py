#!/usr/bin/env python3
import os
import sys
import numpy as np
sys.path.append(os.getcwd()+'/src')
from DogSys import DogSys

if __name__ == '__main__':
    # dictionary for configuration
    # dt for Euler integration
    configDict = {"dt": 0.1}

    buildFlag = True
    # buildFlag = False

    # initialize DogSys
    MyDog = DogSys(configDict=configDict, buildFlag=buildFlag)

    x0 = np.array([0, 0, 0])
    u0= np.array([1.0, 0.1, 0.0])

    x0_dot = MyDog._contDynFun(x0, u0)
    x1 = MyDog._discDynFun(x0, u0)
    print("x0_dot: ", x0_dot)
    print("x1: ", x1)

