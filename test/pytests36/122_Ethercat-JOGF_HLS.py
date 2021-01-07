#!/usr/bin/env python
#

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:2]
tc_no_base = int(os.path.basename(__file__)[0:2]) * 10
###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    jvel = axisCom.get(".JVEL")

    margin = 1.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos = hlm - jvel - margin

    msta = int(axisCom.get(".MSTA"))
    velo = axisCom.get(".VELO")
    accl = axisCom.get(".ACCL")

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} llm={llm:f} hlm={hlm:f} jog_start_pos={jog_start_pos:f}"
    )

    # Make sure that motor is homed
    def test_TC_1221(self):
        tc_no = tc_no_base + 1
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.powerOnHomeAxis(tc_no)
            self.msta = int(self.axisCom.get(".MSTA"))
            passed = (self.msta & self.axisMr.MSTA_BIT_HOMED) == 0
            if not passed:
                self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
                self.assertEqual(
                    passed,
                    True,
                    "MSTA.homed (Axis is not homed)",
                )
            else:
                self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))

    # high limit switch
    def test_TC_1222(self):
        tc_no = tc_no_base + 2
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
            old_high_limit = self.axisCom.get(".HLM")
            old_low_limit = self.axisCom.get(".LLM")
            rdbd = self.axisCom.get(".RDBD")
            self.axisCom.put(".STOP", 1)
            # Go away from limit switch
            self.axisMr.moveWait(tc_no, self.jog_start_pos)
            destination = self.axisCom.get(".HLM")
            rbv = self.axisCom.get(".RBV")
            jvel = self.axisCom.get(".JVEL")
            timeout = self.axisMr.calcTimeOut(destination, jvel) * 2

            self.axisMr.setSoftLimitsOff(tc_no)
            self.axisMr.jogDirection(tc_no, 1)
            # Get values, check them later
            lvio = int(self.axisCom.get(".LVIO"))
            mstaE = int(self.axisCom.get(".MSTA"))
            # Go away from limit switch
            self.axisMr.moveWait(tc_no, old_high_limit - rdbd)
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} msta={mstaE:x} lvio={int(lvio)}"
            )

            self.axisMr.setSoftLimitsOn(old_low_limit, old_high_limit)

            self.assertEqual(
                0,
                mstaE & self.axisMr.MSTA_BIT_PROBLEM,
                "MSTA.Problem should not be set",
            )
            self.assertEqual(
                0, mstaE & self.axisMr.MSTA_BIT_MINUS_LS, "LLS should not be active"
            )
            self.assertNotEqual(
                0, mstaE & self.axisMr.MSTA_BIT_PLUS_LS, "HLS should be active"
            )
        self.axisCom.put("-DbgStrToLOG", "Finish " + str(tc_no))
