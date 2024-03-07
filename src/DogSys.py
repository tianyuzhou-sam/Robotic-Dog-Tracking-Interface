#!/usr/bin/env python3
import sys
import shutil
import subprocess
import casadi as ca
import numpy as np


class DogSys:
    configDict: dict  # a dictionary for parameters
    dimStates: int  # dimension of states
    dimInputs: int  # dimension of inputs

    def __init__(self, configDict: dict, buildFlag=True):
        self.configDict = configDict
        self.dt = float(self.configDict["dt"])
        self.strLib = "DogSys"
        self.prefixBuild = "build/"

        self.dimStates = 3
        self.dimInputs = 3

        states = ca.SX.sym("x", self.dimStates)
        inputs = ca.SX.sym("u", self.dimInputs)

        # Continuous-time Dynamics:
        #     x_dot = f(x,u) [vxcos(theta)-vysin(theta), vxsin(theta)+vycos(theta), w]
        #     u = [u1; u2] = [vx; vy; w]


        # continuous-time dynamical function in casadi SX
        _contDyn = ca.SX.zeros(3, 1)
        _contDyn[0, 0] = inputs[0]*np.cos(states[2]) - inputs[1]*np.sin(states[2])
        _contDyn[1, 0] = inputs[0]*np.sin(states[2]) + inputs[1]*np.cos(states[2])
        _contDyn[2, 0] = inputs[2]

        _discDyn = states + self.dt * _contDyn

        # Acceleration function
        prevInputs = ca.SX.sym("uPrev", self.dimInputs)
        _linearAccX = (inputs[0]-prevInputs[0]) / self.dt
        _linearAccY = (inputs[1]-prevInputs[1]) / self.dt
        _angularAcc = (inputs[2]-prevInputs[2]) / self.dt

        # in casadi.Function
        self._contDynFun = ca.Function("contDynFun", [states, inputs], [_contDyn])
        self._discDynFun = ca.Function("discDynFun", [states, inputs], [_discDyn])
        self._linearAccFunX = ca.Function("linearAccFunX", [prevInputs, inputs], [_linearAccX])
        self._linearAccFunY = ca.Function("linearAccFunY", [prevInputs, inputs], [_linearAccY])
        self._angularAccFun = ca.Function("angularAccFun", [prevInputs, inputs], [_angularAcc])

        # build casadi Function if True
        if buildFlag:
            self.build()

            # load Function from library
            libName = self.prefixBuild + self.strLib + ".so"
            self.contDynFun = ca.external("contDynFun", libName)
            self.discDynFun = ca.external("discDynFun", libName)

    def build(self):
        """
        Generate C codes and compile them.
        """
        # convert casadi.Function to C codes
        codeGen = ca.CodeGenerator(self.strLib + ".c")
        codeGen.add(self._contDynFun)
        codeGen.add(self._discDynFun)

        codeGen.generate(self.prefixBuild)

        # if OS is linux, compatible with Python <= 3.3
        if sys.platform == "linux" or sys.platform == "linux2":
            compiler = "gcc"
        # if OS is MacOS
        elif sys.platform == "darwin":
            compiler = "clang"
        else:
            raise Exception("Only supports Linux or MacOS!")

        # compile
        cmd_args = compiler + " -fPIC -shared -O3 " + self.prefixBuild + self.strLib + ".c -o " + self.prefixBuild + self.strLib + ".so"
        print("Run the following command:")
        print(cmd_args)
        subprocess.run(cmd_args, shell=True, check=True, capture_output=True)

    def forwardPropagate(self, initialState, uAll, timeStepArray, stepNumHorizon, tNow=0.0):
        """
        Forward propagate the dynamics given an initial condition and a sequence of inputs.

        Input:
            initialState: 1d numpy array, the initial state
            uAll: 1d numpy array, the sequence of inputs
                [u_0(0),u_1(0), u_0(1),u_1(1), ..., u_0(T-1),u_1(T-1)]
            timeStep: float, length of discrete time step

        Output:
            timeTraj: 1d numpy array, [0, timeStep, 2*timeStep, ..., timeHorizon]

            xTraj: 2d numpy array, each row is the state at a time step
                [[state_0(0), state_1(0), state_2(0), ...],
                [state_0(1), state_1(1), state_2(1), ...],
                ...
                [state_0(T), state_1(T), state_2(T), ...]]

            uTraj: 2d numpy array, each row is the input at a time step
                [[u_0(0), u_1(0), ...],
                [u_0(1), u_1(1), ...],
                ...
                [u_0(T-1), u_1(T-1), ...]]
        """
        timeTraj = np.zeros(stepNumHorizon+1)
        xTraj = np.zeros((stepNumHorizon+1, self.dimStates))
        uTraj = np.zeros((stepNumHorizon, self.dimInputs))

        xNow = initialState
        timeTraj[0] = tNow  # starting time [sec]
        xTraj[0, :] = np.array(initialState)  # initial state
        for idx in range(stepNumHorizon):
            uNow = uAll[self.dimInputs*idx : self.dimInputs*(idx+1)]
            xNext = self.discDynFun(xNow, uNow)
            timeTraj[idx+1] = timeTraj[idx] + timeStepArray[idx]  # time [sec]

            # casadi array to 1d numpy array
            xTraj[idx+1, :] = np.array(xNext).reshape((1,-1)).flatten()
            uTraj[idx, :] = np.array(uNow)  # input
            xNow = xNext
        return timeTraj, xTraj, uTraj


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


