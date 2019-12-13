#!/usr/bin/env python
#
import unittest
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

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    # Set and readback Vel
    def test_TC_1501(self):
        motor = self.motor
        tc_no = "TC-1501"
        velRB = capv_lib.capvget(motor + '-Vel-RB')
        newVel = velRB + 1.0
        lib.setValueOnSimulator(motor, tc_no, "fVelocity", newVel)
        maxTime = 20 / polltime
        testPassed = False
        while maxTime > 0:
            newVelRB = capv_lib.capvget(motor + '-Vel-RB')
            print('%s:%d newVel=%f newVelRB=%f' % (tc_no, lineno(), newVel, newVelRB))

            if newVelRB == newVel:
                testPassed = True
                maxTime = 0
            else:
                time.sleep(polltime)
                maxTime = maxTime - polltime

        # restore the original value
        lib.setValueOnSimulator(motor, tc_no, "fVelocity", velRB)
        assert(testPassed)
