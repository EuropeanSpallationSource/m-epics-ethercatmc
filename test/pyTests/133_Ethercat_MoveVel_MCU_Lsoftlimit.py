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
    print("url_string=%s" % (url_string))

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    jvel = axisCom.get(".JVEL")

    margin = 1.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos = llm + jvel + margin

    msta = int(axisCom.get(".MSTA"))

    print("llm=%f hlm=%f jog_start_pos=%f" % (llm, hlm, jog_start_pos))

    # Assert that motor is homed
    def test_TC_1331(self):
        tc_no = "TC-1331"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # per10 UserPosition
    def test_TC_1332(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1332-10-percent-UserPosition"
            print("%s" % tc_no)
            done = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f done=%s"
                % (tc_no, UserPosition, self.jog_start_pos, done)
            )
            self.assertEqual(1, done, "moveWait should return done")

    # Low soft limit in controller when using MoveVel
    def test_TC_1333(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1333-low-soft-limit-MoveVel"
            print("%s" % tc_no)

            jar = self.axisCom.get(".JAR")
            self.axisCom.put("-ACCS", jar)

            rbv = self.axisCom.get(".RBV")
            destination = self.axisCom.get(".LLM") - 1
            jvel = self.axisCom.get(".JVEL")
            timeout = self.axisMr.calcTimeOut(destination, jvel)
            print(
                "%s rbv=%f destination=%f timeout=%f"
                % (tc_no, rbv, destination, timeout)
            )
            res = self.axisCom.put("-MoveVel", 0 - jvel)
            # TODO: The -MoveVel PV is not always there ?
            # Investigations needed
            # if (res == None):
            #    print('%s caput -MoveVel res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -MoveVel retuned not None. PV not found ?')
            # else:
            #    print('%s caput -MoveVel res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -MoveVel returned 1')

            done = self.axisMr.waitForStartAndDone(tc_no, timeout)

            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))

            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                self.axisMr.resetAxis(tc_no)

            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_MINUS_LS,
                "DLY Minus hard limit not reached MoveVel",
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PLUS_LS,
                "DLY Plus hard limit not reached MoveVel",
            )
            self.assertEqual(0, miss, "DLY MISS not set MoveVel")

    # per10 UserPosition
    def test_TC_1334(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1234-10-percent-UserPosition"
            print("%s" % tc_no)
            done = self.axisMr.moveWait(tc_no, self.jog_start_pos)
            UserPosition = self.axisCom.get(".RBV", use_monitor=False)
            print(
                "%s postion=%f jog_start_pos=%f done=%s"
                % (tc_no, UserPosition, self.jog_start_pos, done)
            )
            self.assertEqual(1, done, "moveWait should return done")

    # Low soft limit in controller when using MoveAbs
    def test_TC_1335(self):
        if self.msta & self.axisMr.MSTA_BIT_HOMED:
            tc_no = "TC-1235-low-soft-limit Moveabs"
            print("%s" % tc_no)
            drvUseEGU = self.axisCom.get("-DrvUseEGU-RB")
            if drvUseEGU == 1:
                mres = 1.0
            else:
                mres = self.axisCom.get(".MRES")
            rbv = self.axisCom.get(".RBV")

            jar = self.axisCom.get(".JAR")
            self.axisCom.put("-ACCS", jar / mres)

            jvel = self.axisCom.get(".JVEL")
            self.axisCom.put("-VELO", jvel / mres)

            destination = self.llm - 1
            timeout = self.axisMr.calcTimeOut(destination, jvel)
            print(
                "%s: rbv=%f destination=%f timeout=%f"
                % (tc_no, rbv, destination, timeout)
            )

            res = self.axisCom.put("-MoveAbs", (destination) / mres)
            # if (res == None):
            #    print('%s caput -Moveabs res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -Moveabs retuned not None. PV not found ?')
            # else:
            #    print('%s caput -Moveabs res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -Moveabs returned 1')

            done = self.axisMr.waitForStartAndDone(tc_no, timeout)

            msta = int(self.axisCom.get(".MSTA"))
            miss = int(self.axisCom.get(".MISS"))
            done = self.axisMr.verifyRBVinsideRDBD(tc_no, destination)

            if msta & self.axisMr.MSTA_BIT_PROBLEM:
                self.axisMr.resetAxis(tc_no)
            # TODO: Check error; errorId
            # self.assertEqual(True, done, "verifyRBVinsideRDBD returns true Moveabs")

            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_MINUS_LS,
                "DLY Minus hard limit not reached Moveabs",
            )
            self.assertEqual(
                0,
                msta & self.axisMr.MSTA_BIT_PLUS_LS,
                "DLY Plus hard limit not reached Moveabs",
            )
            self.assertEqual(0, miss, "DLY MISS not set Moveabs")
