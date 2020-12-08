#!/usr/bin/env python
#

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = "121xx.py"
###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    saved_DLY = axisCom.get(".DLY")
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    jvel = axisCom.get(".JVEL")

    margin = 1.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos = hlm - jvel - margin

    msta = int(axisCom.get(".MSTA"))

    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} llm={llm:f} hlm={hlm:f} jog_start_pos={jog_start_pos:f}")

    # Make sure that motor is homed
    def test_TC_1211(self):
        tc_no = "1211"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.powerOnHomeAxis(tc_no)
            self.msta = int(self.axisCom.get(".MSTA"))
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # close-toHLM
    def test_TC_1212(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1212"
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f"
                % (tc_no, UserPosition, self.jog_start_pos)
            )

    # High soft limit JOGF
    def test_TC_1213(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1213"
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisCom.put(".DLY", 1.0)
            self.axisMr.jogDirection(tc_no, 1)
            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            self.axisCom.put(".DLY", self.saved_DLY)
            self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
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
            self.assertEqual(1, lvio, "DLY JOGF should have LVIO set")

    # close-toHLM
    def test_TC_1214(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1214"
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f"
                % (tc_no, UserPosition, self.jog_start_pos)
            )

    def test_TC_1215(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1215"
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisCom.put(".DLY", 0.0)
            self.axisMr.jogDirection(tc_no, 1)
            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            self.axisCom.put(".DLY", self.saved_DLY)
            self.axisCom.put(".JOGF", 0)
            resW = self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
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
            self.assertEqual(1, lvio, "ndly JOGF should have LVIO set")

    # High soft limt JOGF
    def test_TC_1216(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1216"
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisCom.put(".DLY", 0.0)
            mip1 = int(self.axisCom.get(".MIP"))
            self.axisMr.jogDirection(tc_no, 1)

            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))
            self.axisMr.waitForMipZero(tc_no, self.saved_DLY)
            mip2 = int(self.axisCom.get(".MIP"))
            jogf = int(self.axisCom.get(".JOGF"))

            self.axisCom.put(".DLY", self.saved_DLY)
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} mip1={mip1:x} mip2={mip2:x}")

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
            self.assertEqual(0, jogf, "ndly2 MIP1 not set JOGF")
            self.assertEqual(1, lvio, "ndly2 should have LVIO set")

    # close-toHLM UserPosition
    def test_TC_1217(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1217"
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f"
                % (tc_no, UserPosition, self.jog_start_pos)
            )

    # High soft limit JOGR + DIR
    def test_TC_1218(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "1218"
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            saved_DIR = self.axisCom.get(".DIR")
            saved_FOFF = self.axisCom.get(".FOFF")
            self.axisCom.put(".FOFF", 1)
            self.axisCom.put(".DIR", 1)
            self.axisMr.jogDirection(tc_no, 0)

            lvio = int(self.axisCom.get(".LVIO"))
            msta = int(self.axisCom.get(".MSTA"))

            self.axisCom.put(".JOGF", 0)
            self.axisCom.put(".DIR", saved_DIR)
            self.axisCom.put(".FOFF", saved_FOFF)

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
                "Plus hard limit not reached JOGF DIR",
            )
