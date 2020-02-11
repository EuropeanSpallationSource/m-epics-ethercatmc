#!/usr/bin/env python
#
import unittest
import math
import os
import sys
import time
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
import inspect
def lineno():
    return inspect.currentframe().f_back.f_lineno
###

polltime = 0.1

def setAndReadBackParam(self, motor, tc_no, pvSuffix, paramInSimu):
    pvname = motor + pvSuffix
    valRB = capv_lib.capvget(pvname)
    newVal = valRB + 1
    lib.setValueOnSimulator(motor, tc_no, paramInSimu, newVal)
    maxTime = 30 # 30 seconds maximum to poll all parameters
    testPassed = False
    maxDelta = math.fabs(newVal) * 0.02 # 2 % error tolerance margin
    while maxTime > 0:
        newValRB = capv_lib.capvget(pvname)
        print('%s:%d %s newVal=%f newValRB=%f' % (tc_no, lineno(), pvname, newVal, newValRB))

        if lib.calcAlmostEqual(motor, tc_no, newVal, newValRB, maxDelta):
            testPassed = True
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime

    # restore the original value
    lib.setValueOnSimulator(motor, tc_no, paramInSimu, valRB)
    assert(testPassed)


class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    # Set and readback Vel
    def test_TC_1501(self):
        motor = self.motor
        tc_no = "TC-1501"
        setAndReadBackParam(self, motor, tc_no, '-Vel-RB', 'fVelocity')

   # Set and readback Acc
    def test_TC_1502(self):
        motor = self.motor
        tc_no = "TC-1502"
        setAndReadBackParam(self, motor, tc_no, '-Acc-RB', 'fAcceleration')

    # Set and readback high soft limit value
    def test_TC_1503(self):
        motor = self.motor
        tc_no = "TC-1503"
        setAndReadBackParam(self, motor, tc_no, '-CfgDHLM-RB', 'fHighSoftLimitPos')

    ## Set and readback high soft limit enable
    # Cant run those, PILS has no enable bit
    #def test_TC_1504(self):
    #    motor = self.motor
    #    tc_no = "TC-1504"
    #    setAndReadBackParam(self, motor, tc_no, '-CfgDHLM-En-RB', 'bEnableHighSoftLimit')

    # Set and readback low soft limit value
    def test_TC_1505(self):
        motor = self.motor
        tc_no = "TC-1505"
        setAndReadBackParam(self, motor, tc_no, '-CfgDLLM-RB', 'fLowSoftLimitPos')

    ## Set and readback low soft limit enable
    # Cant run those, PILS has no enable bit
    #def test_TC_1506(self):
    #    motor = self.motor
    #    tc_no = "TC-1506"
    #    setAndReadBackParam(self, motor, tc_no, '-CfgDLLM-En-RB', 'bEnableLowSoftLimit')
