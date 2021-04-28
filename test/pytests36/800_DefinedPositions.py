#!/usr/bin/env python
#

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]
tc_no_base = int(os.path.basename(__file__)[0:3]) * 10
###


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=True)
    motorPvName = axisCom.getMotorPvName()
    axisMr = AxisMr(axisCom)
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")

    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}")

    # Make sure that motor is homed
    def test_TC_8001(self):
        tc_no = tc_no_base + 1
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        msta = int(self.axisCom.get(".MSTA"))
        if not (msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.powerOnHomeAxis(tc_no)
            msta = int(self.axisCom.get(".MSTA"))
            passed = (msta & self.axisMr.MSTA_BIT_HOMED) != 0
            if not passed:
                self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
                self.assertEqual(
                    passed,
                    True,
                    "MSTA.homed (Axis is not homed)",
                )
            else:
                self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)

    # calculate values
    def test_TC_8002(self):
        msta = int(self.axisCom.get(".MSTA"))
        if msta & self.axisMr.MSTA_BIT_HOMED:
            old_dol = self.axisCom.get(".DOL")
            old_omsl = self.axisCom.get(".OMSL")
            self.axisCom.put(".DOL", self.motorPvName + "-DefPosSEL1")
            self.axisCom.put(".OMSL", "closed_loop")
            velo = self.axisCom.get(".VELO")

            # Calculate values
            steps = 12
            step = 0
            passed = True

            while step < steps and passed:
                tc_no = 100 * (tc_no_base + 2) + step
                self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)))
                pvname = "-DefPosVAL" + chr(ord("A") + step)
                destination = (step * self.hlm + (steps - step) * self.llm) / steps
                self.axisCom.put(pvname, destination)
                timeout = self.axisMr.calcTimeOut(destination, velo)
                self.axisCom.put("-DefPosSEL", step, wait=True, timeout=timeout)
                step = step + 1

                # passed =
                if passed:
                    self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
                else:
                    self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
            self.axisCom.put(".DOL", old_dol)
            self.axisCom.put(".OMSL", old_omsl)
            assert passed


#    # calculate values
#    def test_TC_8003(self):
#        tc_no = tc_no_base + 3
#        hlm = axisCom.get(".HLM")
#        llm = axisCom.get(".LLM")
#
#        msta = int(self.axisCom.get(".MSTA"))
#        if msta & self.axisMr.MSTA_BIT_HOMED:
#            self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)))
#            passed = self.axisMr.moveIntoLS(tc_no=tc_no, direction=direction)
#            if passed:
#                self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
#            else:
#                self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
#            assert passed
#
#
