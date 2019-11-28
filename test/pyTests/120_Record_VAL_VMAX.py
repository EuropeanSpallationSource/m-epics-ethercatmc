#!/usr/bin/env python
#

import math
import unittest
import os
import sys
from motor_lib import motor_lib
lib = motor_lib()
from motor_globals import motor_globals
globals = motor_globals()
import capv_lib
###

def motorPositionTC(self, motor, tc_no, destination, velocity):
    if (self.msta & lib.MSTA_BIT_HOMED):
        #capv_lib.capvput(motor + '-DbgStrToLOG', "Start TC " + tc_no[0:20]);
        if velocity != self.velo:
            capv_lib.capvput(motor + '.VELO', velocity)

        timeout = lib.calcTimeOut(motor, destination, velocity)
        res = lib.move(motor, destination, timeout)
        if velocity != self.velo:
            capv_lib.capvput(motor + '.VELO', self.velo)

        UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
        print('%s postion=%f destination=%f' % (
            tc_no, UserPosition, destination))
        self.assertEqual(res, globals.SUCCESS, 'move returned SUCCESS')
        res2 = lib.postMoveCheck(motor)
        self.assertEqual(res2, globals.SUCCESS, 'postMoveCheck returned SUCCESS')


class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    #capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__))

    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')
    velo = capv_lib.capvget(motor + '.VELO')
    vmax = capv_lib.capvget(motor + '.VMAX')
    if vmax == 0.0:
        vmax = velo * 100.0
    msta             = int(capv_lib.capvget(motor + '.MSTA'))
    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    def test_TC_1200(self):
        motor = self.motor
        tc_no = "TC-1200"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')

    def test_TC_1201(self):
        motor = self.motor
        tc_no = "TC-1201"
        motorPositionTC(self, motor, tc_no, self.llm, self.velo)


    def test_TC_1202(self):
        motor = self.motor
        tc_no = "TC-1202"
        motorPositionTC(self, motor, tc_no, self.per10_UserPosition, self.velo)


    def test_TC_1203(self):
        motor = self.motor
        tc_no = "TC-1203"
        motorPositionTC(self, motor, tc_no, self.per90_UserPosition, self.velo)


    def test_TC_1204(self):
        motor = self.motor
        tc_no = "TC-1204"
        motorPositionTC(self, motor, tc_no, self.hlm, self.velo)

    def test_TC_1205(self):
        motor = self.motor
        tc_no = "TC-1205"
        motorPositionTC(self, motor, tc_no, self.per90_UserPosition, self.vmax)

    def test_TC_1206(self):
        motor = self.motor
        tc_no = "TC-1206"
        motorPositionTC(self, motor, tc_no, self.per10_UserPosition, self.vmax)

    def test_TC_1207(self):
        motor = self.motor
        tc_no = "TC-1207"
        motorPositionTC(self, motor, tc_no, self.llm, self.vmax)
