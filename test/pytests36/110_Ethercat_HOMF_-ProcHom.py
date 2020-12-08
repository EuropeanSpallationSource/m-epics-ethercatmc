#!/usr/bin/env python

#
# Test differen homing procedures
#


import datetime
import unittest
import os
import sys
import math
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = "110xx.py"

###

START_FROM_LLS = -1
START_FROM_MID = 0
START_FROM_HLS = 1


def homeTheMotor(self, tc_no, homProc, jogToLSBefore):
    old_high_limit = self.axisCom.get(".HLM")
    old_low_limit = self.axisCom.get(".LLM")
    old_HomProc = self.axisCom.get("-HomProc")
    old_HomPos = self.axisCom.get("-HomPos")

    if jogToLSBefore != 0:
        msta = int(self.axisCom.get(".MSTA"))
        if msta & self.axisMr.MSTA_BIT_HOMED and old_high_limit > old_low_limit:
            # if we are homed, move absolute to the soft limit
            # This is faster than jogging
            if jogToLSBefore > 0:
                self.axisMr.moveWait(tc_no, old_high_limit)
            else:
                self.axisMr.moveWait(tc_no, old_low_limit)

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
        self.axisCom.put("-HomProc", homProc)
        if homProc == 1:
            self.axisCom.put("-HomPos", old_low_limit - 1.0)
        elif homProc == 2:
            self.axisCom.put("-HomPos", old_high_limit + 1.0)

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
    if homProc != 0:
        self.axisCom.put("-HomProc", old_HomProc)
        self.axisCom.put("-HomPos", old_HomPos)

    self.assertEqual(
        0,
        msta2 & self.axisMr.MSTA_BIT_SLIP_STALL,
        tc_no + "MSTA.no MSTA_BIT_SLIP_STALL",
    )
    self.assertNotEqual(0, homed, tc_no + "MSTA.homed (Axis has been homed)")


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
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_11100(self):
        tc_no = "11100"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 0, START_FROM_LLS)

    def test_TC_11101(self):
        tc_no = "11101"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 0, START_FROM_MID)

    def test_TC_11102(self):
        tc_no = "11112"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 0, START_FROM_HLS)

    def test_TC_11110(self):
        tc_no = "11110"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 1, START_FROM_LLS)

    def test_TC_11111(self):
        tc_no = "11111"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 1, START_FROM_MID)

    def test_TC_11112(self):
        tc_no = "11112"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 1, START_FROM_HLS)

    def test_TC_11120(self):
        tc_no = "11120"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 2, START_FROM_LLS)

    def test_TC_11121(self):
        tc_no = "11121"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 2, START_FROM_MID)

    def test_TC_11122(self):
        tc_no = "11122"
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor")
        homeTheMotor(self, tc_no, 2, START_FROM_HLS)
