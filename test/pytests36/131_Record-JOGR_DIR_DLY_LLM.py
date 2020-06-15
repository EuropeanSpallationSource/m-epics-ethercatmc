#!/usr/bin/env python
#

import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    saved_DLY = axisCom.get(".DLY")
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    jvel = axisCom.get(".JVEL")

    margin = 1.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos = llm + jvel + margin

    msta = int(axisCom.get(".MSTA"))

    print(f"llm={llm:f} hlm={hlm:f} jog_start_pos={jog_start_pos:f}")

    # Assert that motor is homed
    def test_TC_1311(self):
        tc_no = "TC-1311"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # per10 UserPosition
    def test_TC_1312(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1312-10-percent-UserPosition"
            print(f"{tc_no}")
            done = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f done=%s"
                % (tc_no, UserPosition, self.jog_start_pos, done)
            )
            self.assertEqual(1, done, "moveWait should return done")

    # Low soft limit JOGR
    def test_TC_1313(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1313-low-soft-limit JOGR"
            print(f"{tc_no}")
            self.axisCom.put(".DLY", 1.0)
            done = self.axisMr.jogDirection(tc_no, 0)
            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            self.axisCom.put(".DLY", self.saved_DLY)
            resW = self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
            self.assertEqual(1, done, "DLY JOGF should be done after jogDirection")
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PROBLEM,
                "DLY JOGF should not give MSTA.Problem",
            )
            self.assertEqual(
                0, msta & self.axisMr.MSTA_BIT_MINUS_LS, "DLY JOGF should not reach LLS"
            )
            self.assertEqual(
                0, msta & self.axisMr.MSTA_BIT_PLUS_LS, "DLY JOGF should not reach HLS"
            )
            self.assertEqual(0, miss, "DLY JOGF should not have MISS set")
            self.assertEqual(1, resW, "DLY JOGF should have MIP = 0")
            self.assertEqual(1, lvio, "DLY JOGF should have LVIO set")

    # per10 UserPosition
    def test_TC_1314(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1314-10-percent-UserPosition"
            print(f"{tc_no}")
            done = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f done=%s"
                % (tc_no, UserPosition, self.jog_start_pos, done)
            )
            self.assertEqual(1, done, "moveWait should return done")

    def test_TC_1315(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1315-low-soft-limit JOGR"
            print(f"{tc_no}")
            self.axisCom.put(".DLY", 0.0)
            done = self.axisMr.jogDirection(tc_no, 0)
            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            self.axisCom.put(".DLY", self.saved_DLY)
            self.axisCom.put(".JOGF", 0)
            resW = self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
            self.assertEqual(True, done, "ndly JOGF should be done after jogDirection")
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PROBLEM,
                "ndly JOGF should not give MSTA.Problem",
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_MINUS_LS,
                "ndly JOGF should not reach LLS",
            )
            self.assertEqual(
                0, msta & self.axisMr.MSTA_BIT_PLUS_LS, "ndly JOGF should not reach HLS"
            )
            self.assertEqual(0, miss, "ndly JOGF should not have MISS set")
            self.assertEqual(1, resW, "ndly JOGF should have MIP = 0")
            self.assertEqual(1, lvio, "ndly JOGF should have LVIO set")

    # Low soft limit JOGR
    def test_TC_1316(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1313-low-soft-limit JOGR"
            print(f"{tc_no}")
            self.axisCom.put(".DLY", 0.0)
            mip1 = int(self.axisCom.get(".MIP"))
            done = self.axisMr.jogDirection(tc_no, 0)

            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))
            resW = self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
            mip2 = int(self.axisCom.get(".MIP"))
            jogr = int(self.axisCom.get(".JOGR"))

            self.axisCom.put(".DLY", self.saved_DLY)
            print(f"{tc_no} mip1={mip1:x} mip2={mip2:x}")

            self.assertEqual(True, done, "done should be True after jogDirection")
            self.assertEqual(
                0, msta & self.axisMr.MSTA_BIT_PROBLEM, "ndly2 No MSTA.Problem JOGF"
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_MINUS_LS,
                "ndly2 Minus hard limit not reached JOGF",
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PLUS_LS,
                "ndly2 Plus hard limit not reached JOGF",
            )
            self.assertEqual(0, miss, "ndly2 MISS not set JOGF")
            self.assertEqual(0, mip1, "ndly2 MIP1 not set JOGF")
            self.assertEqual(
                0, mip2 & self.axisMr.MIP_BIT_JOGF, "ndly2 MIP2.JOGF not set JOGF"
            )
            # self.assertEqual(1, resW,                        'ndly1 JOGF not set')
            self.assertEqual(0, jogr, "ndly2 MIP1 not set JOGR")
            self.assertEqual(1, lvio, "ndly2 should have LVIO set")

    # per10 UserPosition
    def test_TC_1317(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1314-10-percent-UserPosition"
            print(f"{tc_no}")
            done = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f done=%s"
                % (tc_no, UserPosition, self.jog_start_pos, done)
            )
            self.assertEqual(1, done, "moveWait should return done")

    # Low soft limit JOGF + DIR
    def test_TC_1318(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1315-low-soft-limit JOGF DIR"
            print(f"{tc_no}")
            saved_DIR = self.axisCom.get(".DIR")
            saved_FOFF = self.axisCom.get(".FOFF")
            self.axisCom.put(".FOFF", 1)
            self.axisCom.put(".DIR", 1)
            done = self.axisMr.jogDirection(tc_no, 1)

            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))

            self.axisCom.put(".DIR", saved_DIR)
            self.axisCom.put(".FOFF", saved_FOFF)

            self.assertEqual(True, done, "done should be True after jogDirection")
            self.assertEqual(
                0, msta & self.axisMr.MSTA_BIT_PROBLEM, "No Error MSTA.Problem JOGF DIR"
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_MINUS_LS,
                "Minus hard limit not reached JOGF DIR",
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PLUS_LS,
                "Plus hard limit not reached JOGR DIR",
            )