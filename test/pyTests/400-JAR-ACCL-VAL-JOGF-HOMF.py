#!/usr/bin/env python

import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print("url_string=%s" % (url_string))

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")

    per10_UserPosition = round((9 * llm + 1 * hlm) / 10)
    per20_UserPosition = round((8 * llm + 2 * hlm) / 10)
    msta = int(axisCom.get(".MSTA"))

    def getAcceleration(self, tc_no):
        print("%s: getAcceleration" % (tc_no))
        res = self.axisCom.get("-Acc-RB")
        return res

    # Assert that motor is homed
    def test_TC_401(self):
        tc_no = "TC-401"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # 10% dialPosition
    def test_TC_402(self):
        tc_no = "TC-402-10-percent"
        print("%s" % tc_no)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            done = self.axisMr.moveWait(tc_no, self.per10_UserPosition)
            self.assertEqual(1, done, "moveWait should return done")

    # 20% dialPosition
    def test_TC_403(self):
        tc_no = "TC-403-20-percent"
        print("%s" % tc_no)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            saved_ACCL = float(self.axisCom.get(".ACCL"))
            saved_VELO = float(self.axisCom.get(".VELO"))
            used_ACCL = saved_ACCL + 1.0  # Make sure we have an acceleration != 0
            self.axisCom.put(".ACCL", used_ACCL)
            done = self.axisMr.moveWait(tc_no, self.per20_UserPosition)
            resacc = self.getAcceleration(tc_no)
            expacc = saved_VELO / used_ACCL
            self.axisCom.put(".ACCL", saved_ACCL)
            print("%s ACCL=%f expacc=%f resacc=%f" % (tc_no, used_ACCL, expacc, resacc))
            assert self.axisMr.calcAlmostEqual(tc_no, expacc, resacc, 2)
            self.assertEqual(1, done, "moveWait should return done")

    # JOGR
    def test_TC_404(self):
        tc_no = "TC-404-JOGR"
        print("%s" % tc_no)
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.put("-DbgStrToLOG", "Start " + tc_no)
            accl = float(self.axisCom.get(".ACCL"))
            jvel = float(self.axisCom.get(".JVEL"))
            saved_JAR = float(self.axisCom.get(".JAR"))
            used_JAR = jvel / (accl + 2.0)
            self.axisCom.put(".JAR", used_JAR)
            self.axisCom.put(".JOGR", 1)
            ret2 = self.axisMr.waitForStart(tc_no, 2.0)

            resacc = self.getAcceleration(tc_no)
            expacc = used_JAR
            self.axisCom.put(".JOGR", 0)
            # Wait depending on JVEL (and positions)
            time_to_wait = (self.per20_UserPosition - self.llm) / jvel + 2 * accl + 1.0
            self.axisMr.waitForStop(tc_no, time_to_wait)
            self.axisCom.put(".JAR", saved_JAR)
            print("%s JAR=%f expacc=%f resacc=%f" % (tc_no, used_JAR, expacc, resacc))

            self.axisCom.put("-DbgStrToLOG", "End " + tc_no)
            assert self.axisMr.calcAlmostEqual(tc_no, expacc, resacc, 2)
