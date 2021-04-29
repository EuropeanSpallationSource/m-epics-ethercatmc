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


def setTPRO(self, tpro):
    self.axisCom.put("-DefPosSEL.TPRO", tpro)
    self.axisCom.put("-DefPosSELCALC1.TPRO", tpro)
    self.axisCom.put("-DefPosSELCALC2.TPRO", tpro)
    self.axisCom.put("-DefPosSetDOL_.TPRO", tpro)
    self.axisCom.put("-DefPosSEL1.TPRO", tpro)
    if tpro == 1:
        self.axisCom.put(".SPAM", 2047)
    else:
        self.axisCom.put(".SPAM", 15)


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
            setTPRO(self, 1)
            # Note: Setting up .DOL and .OMLS is done in the template
            # self.axisCom.put(".DOL", self.motorPvName + "-DefPosSEL1")
            # self.axisCom.put(".OMSL", "closed_loop")
            velo = self.axisCom.get(".VELO")

            # Calculate values
            steps = 12
            step = 0
            passed = True

            while step < steps and passed:
                tc_no = 100 * (tc_no_base + 2) + step
                self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)), wait=True)
                pvname = "-DefPosVAL" + chr(ord("A") + step)
                destination = (
                    float(step) * self.hlm + float(steps - step) * self.llm
                ) / float(steps)
                self.axisCom.put(pvname, destination)
                timeout = self.axisMr.calcTimeOut(destination, velo)
                self.axisCom.put("-DefPosSEL", step, wait=True, timeout=timeout)
                # The following should not be needed:
                # if not self.axisMr.verifyRBVinsideRDBD(tc_no, destination):
                #    try:
                #        self.axisMr.waitForStart(tc_no, 1.0)
                #    except Exception:
                #        pass

                # self.axisMr.waitForStop(tc_no, timeout)

                postMoveCheckOK = self.axisMr.postMoveCheck(tc_no)
                verifyRBVinsideRDBDOK = self.axisMr.verifyRBVinsideRDBD(
                    tc_no, destination
                )
                passed = passed and postMoveCheckOK and verifyRBVinsideRDBDOK
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} step={step} destination={destination} postMoveCheckOK={postMoveCheckOK} verifyRBVinsideRDBDOK={verifyRBVinsideRDBDOK}"
                )

                step = step + 1
                if passed:
                    self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
                else:
                    self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
            self.axisCom.put(".DOL", old_dol)
            self.axisCom.put(".OMSL", old_omsl)
            setTPRO(self, 0)
            assert passed
