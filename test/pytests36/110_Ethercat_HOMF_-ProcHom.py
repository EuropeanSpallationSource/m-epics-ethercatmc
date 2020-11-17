#!/usr/bin/env python

#
# Test differen homing procedures
#


import unittest
import os
import sys
import math
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

###

START_FROM_LLS = -1
START_FROM_MID = 0
START_FROM_HLS = 1


def homeTheMotor(self, tc_no, homProc, jogToLSBefore):
    old_high_limit = self.axisCom.get(".HLM")
    old_low_limit = self.axisCom.get(".LLM")
    if homProc != 0:
        old_HomProc = self.axisCom.get("-HomProc")
    if jogToLSBefore != 0:
        self.axisMr.setSoftLimitsOff(tc_no)
        # soft limit range assumed to be = hard range /1.5 or so
        # It is only needed to calculate a good timeout
        jvel = self.axisCom.get(".JVEL")
        accl = self.axisCom.get(".ACCL")
        time_to_wait = 1.5 * (old_high_limit - old_low_limit) / jvel + 2 * accl

        self.axisMr.jogDirectionTimeout(tc_no, jogToLSBefore, time_to_wait)
        self.axisCom.put(".LLM", old_low_limit)
        self.axisCom.put(".HLM", old_high_limit)
    else:
        self.axisMr.moveWait(tc_no, (old_high_limit + old_low_limit) / 2.0)

    if homProc != 0:
        axisCom.put("-HomProc", homProc)
    msta = int(self.axisCom.get(".MSTA"))
    # We can home while sitting on a limit switch
    if msta & self.axisMr.MSTA_BIT_MINUS_LS:
        self.axisCom.put(".HOMR", 1)
    else:
        self.axisCom.put(".HOMF", 1)

    # Calculate the timeout, based on the driving range
    range_postion = self.axisCom.get(".HLM") - self.axisCom.get(".LLM")
    hvel = self.axisCom.get(".HVEL")
    accl = self.axisCom.get(".ACCL")
    msta1 = int(self.axisCom.get(".MSTA"))

    if range_postion > 0 and hvel > 0:
        time_to_wait = 1 + 2 * range_postion / hvel + 2 * accl
    else:
        time_to_wait = 180

    self.axisMr.waitForStart(tc_no, 3)
    if msta1 & self.axisMr.MSTA_BIT_HOMED:
        unhomed = 0
    else:
        unhomed = 1
    print(
        "%s homeTheMotor msta1=%s unhomed=%d"
        % (tc_no, self.axisMr.getMSTAtext(msta1), unhomed)
    )
    stopped = self.axisMr.waitForStop(tc_no, time_to_wait)

    msta2 = int(self.axisCom.get(".MSTA"))
    homed = 0
    if msta2 & self.axisMr.MSTA_BIT_HOMED:
        homed = 1
    #    print('%s homeTheMotor stopped=%d msta2=%s homed=%d' % \
    #        (tc_no, stopped, self.axisMr.getMSTAtext(msta2), homed))
    # self.assertEqual(True, started,                          tc_no +  "started = True")
    # self.assertEqual(True, stopped,                          tc_no +  "stopped = True")
    self.assertEqual(
        0,
        msta2 & self.axisMr.MSTA_BIT_SLIP_STALL,
        tc_no + "MSTA.no MSTA_BIT_SLIP_STALL",
    )
    self.assertNotEqual(0, homed, tc_no + "MSTA.homed (Axis has been homed)")
    if homProc != 0:
        self.pv_HomProc.put(old_HomProc, wait=True)


def homeLimBwdfromLLS(self, tc_no):
    homeTheMotor(self, tc_no, 1, START_FROM_LLS)


def homeLimBwdfromMiddle(self, tc_no):
    homeTheMotor(self, tc_no, 1, START_FROM_MID)


def homeLimBwdfromHLS(self, tc_no):
    homeTheMotor(self, tc_no, 1, START_FROM_HLS)


def homeLimFwdfromLLS(self, tc_no):
    homeTheMotor(self, tc_no, 2, START_FROM_LLS)


def homeLimFwdfromMiddle(self, tc_no):
    homeTheMotor(self, tc_no, 2, START_FROM_MID)


def homeLimFwdfromHLS(self, tc_no):
    homeTheMotor(self, tc_no, 2, START_FROM_HLS)


def homeSwitchfromLimBwdFromLLS(self, tc_no):
    homeTheMotor(self, tc_no, 3, START_FROM_LLS)


def homeSwitchfromLimBwdFromHLS(self, tc_no):
    homeTheMotor(self, tc_no, 3, START_FROM_HLS)


def homeSwitchfromLimBwdFromMiddle(self, tc_no):
    homeTheMotor(self, tc_no, 3, START_FROM_MID)


def homeSwitchfromLimFwdFromLLS(self, tc_no):
    homeTheMotor(self, tc_no, 4, START_FROM_LLS)


def homeSwitchfromLimFwdFromHLS(self, tc_no):
    homeTheMotor(self, tc_no, 4, START_FROM_HLS)


def homeSwitchfromLimFwdFromMiddle(self, tc_no):
    homeTheMotor(self, tc_no, 4, START_FROM_MID)


def homeSwitchMidfromLimBwdFromLLS(self, tc_no):
    homeTheMotor(self, tc_no, 5, START_FROM_LLS)


def homeSwitchMidfromLimBwdFromHLS(self, tc_no):
    homeTheMotor(self, tc_no, 5, START_FROM_HLS)


def homeSwitchMidfromLimBwdFromMiddle(self, tc_no):
    homeTheMotor(self, tc_no, 5, START_FROM_MID)


def homeSwitchMidfromLimFwdFromLLS(self, tc_no):
    homeTheMotor(self, tc_no, 6, START_FROM_LLS)


def homeSwitchMidfromLimFwdFromHLS(self, tc_no):
    homeTheMotor(self, tc_no, 6, START_FROM_HLS)


def homeSwitchMidfromLimFwdFromMiddle(self, tc_no):
    homeTheMotor(self, tc_no, 6, START_FROM_MID)


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_11100(self):
        tc_no = "11100"
        print(f"{tc_no} Home the motor")
        homeTheMotor(self, tc_no, 0, START_FROM_LLS)

    def test_TC_11101(self):
        tc_no = "11101"
        print(f"{tc_no} Home the motor")
        homeTheMotor(self, tc_no, 0, START_FROM_MID)

    def test_TC_11102(self):
        tc_no = "11111"
        print(f"{tc_no} Home the motor")
        homeTheMotor(self, tc_no, 0, START_FROM_HLS)


#    def test_TC_11112(self):
#        tc_no = "11112"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeLimFwdfromLLS(self, tc_no)
#
#    def test_TC_11120(self):
#        tc_no = "11120"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeLimFwdfromMiddle(self, tc_no)
#
#    def test_TC_11121(self):
#        tc_no = "11121"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeLimFwdfromHLS(self, tc_no)
#
#    def test_TC_11122(self):
#        tc_no = "11122"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeLimBwdfromHLS(self, tc_no)
#
#    def test_TC_11130(self):
#        tc_no = "11130"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeSwitchfromLimFwdFromLLS(self, tc_no)
#
#    def test_TC_11131(self):
#        tc_no = "11131"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeSwitchfromLimFwdFromMiddle(self, tc_no)
#
#    def test_TC_11132(self):
#        tc_no = "11132"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeSwitchfromLimBwdFromLLS(self, tc_no)
#
#    def test_TC_11140(self):
#        tc_no = "11140"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeSwitchfromLimBwdFromMiddle(self, tc_no)
#
#    def test_TC_11141(self):
#        tc_no = "11141"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeSwitchfromLimBwdFromHLS(self, tc_no)
#
#    def test_TC_11142(self):
#        tc_no = "11142"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeSwitchfromLimFwdFromHLS(self, tc_no)
#
##    def test_TC_11150(self):
##        tc_no = "11150"
##        print(f"{tc_no} Home the motor")
##        if self.pv_HomProc != None:
##            homeSwitchMidfromLimBwdFromMiddle(self, tc_no)
##
##    def test_TC_11151(self):
##        tc_no = "11151"
##        print(f"{tc_no} Home the motor")
##        if self.pv_HomProc != None:
##            homeSwitchMidfromLimBwdFromHLS(self, tc_no)
##
##    def test_TC_11152(self):
##        tc_no = "11152"
##        print(f"{tc_no} Home the motor")
##        if self.pv_HomProc != None:
##            homeSwitchMidfromLimFwdFromHLS(self, tc_no)
##
##    def test_TC_11160(self):
##        tc_no = "11160"
##        print(f"{tc_no} Home the motor")
##        homeSwitchMidfromLimFwdFromLLS(self, tc_no)
##
##    def test_TC_11161(self):
##        tc_no = "11161"
##        print(f"{tc_no} Home the motor")
##        if self.pv_HomProc != None:
##            homeSwitchMidfromLimFwdFromMiddle(self, tc_no)
##
##    def test_TC_11162(self):
##        tc_no = "11162"
##        print(f"{tc_no} Home the motor")
##        if self.pv_HomProc != None:
##            homeSwitchMidfromLimBwdFromLLS(self, tc_no)
#
#    # Need to home with the original homing procedure
#    def test_TC_11191(self):
#        tc_no = "11191"
#        print(f"{tc_no} Home the motor")
#        if self.pv_HomProc != None:
#            homeTheMotor(self,  tc_no, self.old_HomProc, START_FROM_MID)
