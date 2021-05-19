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
            # Note: Setting up .DOL and .OMLS is done in the template
            velo = self.axisCom.get(".VELO")

            # Calculate values
            steps = 12
            step = 0
            passed = True

            while step < steps and passed:
                tc_no = 100 * (tc_no_base + 2) + step
                self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)), wait=True)
                pvname = "-DefPosVAL" + chr(ord("A") + step)
                if step == 0:
                    destination = self.llm
                elif step == steps -1:
                    destination = self.hlm
                else:
                    destination = (
                        float(step) * self.hlm + float(steps - step) * self.llm
                    ) / float(steps - 1)
                self.axisCom.put(pvname, destination)
                # Workaround: Acceleration may be wrong Allow longer timeout
                timeout = 2 + 2 * self.axisMr.calcTimeOut(destination, velo)
                # The -DefPosSEL does not wait on a callback
                # self.axisCom.put("-DefPosSEL", step, wait=True, timeout=timeout)
                self.axisCom.put("-DefPosSEL", step)
                # The following is needed (because there is no callback)
                if not self.axisMr.verifyRBVinsideRDBD(tc_no, destination):
                    try:
                        self.axisMr.waitForStart(tc_no, 1.0)
                    except Exception:
                        pass
                self.axisMr.waitForStop(tc_no, timeout)

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

            self.axisCom.put("-DefPosSEL", -1)

            assert passed
