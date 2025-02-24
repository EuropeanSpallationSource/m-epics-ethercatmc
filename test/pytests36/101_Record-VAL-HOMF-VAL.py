#!/usr/bin/env python

#
# Test homing via EPICS motorRecord
#

import datetime
import math
import inspect
import unittest
import os
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]

polltime = 0.2


def lineno():
    return inspect.currentframe().f_back.f_lineno


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )
    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # Home the motor
    def test_TC_10100(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        tc_no = 10100
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
        )
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)

        self.axisMr.setCNENandWait(tc_no, 1)
        time_to_wait = axisMr.calcHomeTimeOut(tc_no)
        msta = int(axisCom.get(".MSTA"))
        # If we are sitting on the High limit switch, use HOMR
        if msta & axisMr.MSTA_BIT_PLUS_LS:
            axisCom.put(".HOMR", 1)
        else:
            axisCom.put(".HOMF", 1)
        axisMr.waitForStartAndDone(tc_no, time_to_wait)

        # Wait a little bit, motor/master reports "done" already when hitting a limit switch
        # Give the "homed" bit a chance to ripple through the poller
        while time_to_wait > 0:
            msta = int(axisCom.get(".MSTA"))
            print(f"msta={self.axisMr.getMSTAtext(msta)}")
            time.sleep(polltime)
            time_to_wait -= polltime
            if msta & axisMr.MSTA_BIT_HOMED:
                time_to_wait = 0
        testPassed = msta & axisMr.MSTA_BIT_HOMED
        if testPassed:
            self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
        assert testPassed

    # Move the motor by writing to the .VAL field,
    # Home while moving, change .VAL field again
    def test_TC_10101(self):
        axisCom = self.axisCom
        axisMr = self.axisMr
        tc_no = 10101
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        testPassed = True
        oldSPAM = axisMr.getFieldSPAM(tc_no)
        axisMr.setFieldSPAM(tc_no, -1)
        rbv = float(axisCom.get(".RBV"))
        hlm = float(axisCom.get(".HLM"))
        llm = float(axisCom.get(".LLM"))
        if llm >= hlm:
            off = float(axisCom.get(".OFF"))
            dir = float(axisCom.get(".DIR"))
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} off={off:.2f} dir={dir}"
            )
            if off == 0.0 and dir == 0:
                try:
                    lll = float(axisCom.get("-CfgDLLM-RB"))
                    hhh = float(axisCom.get("-CfgDHLM-RB"))
                    llm = lll
                    hlm = hhh
                except:  # noqa: E722
                    print(
                        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} can not get soft limits"
                    )
                    testPassed = False
        if testPassed:
            old_velo = float(axisCom.get(".VELO"))
            if old_velo <= 0.0:
                testPassed = False
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} old_velo={old_velo:.2f} testPassed={testPassed}"
                )
        if testPassed:
            # Figure out the soft limit position far away
            # We will never go there
            if (hlm - rbv) > (rbv - llm):
                pos_far_away = hlm
            else:
                pos_far_away = llm
            pos_to_reach = (hlm + llm) / 2.0
            # The velocity should not be too high to make sense
            # for this test
            maximal_velo = math.fabs(pos_to_reach - rbv) / 8.0
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} old_velo={old_velo:.2f} maximal_velo={testPassed:.2f}"
            )
            if old_velo > maximal_velo:
                axisCom.put(".VELO", maximal_velo)
                velo = maximal_velo
            else:
                velo = old_velo

            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} hlm={hlm:.2f} llm={llm:.2f} rbv={rbv:.2f} pos_far_away={pos_far_away:.2f}"
            )
            if llm >= hlm:
                testPassed = False

        if testPassed:
            axisCom.put(".VAL", pos_far_away)
            axisMr.waitForStart(tc_no, 3)
            time.sleep(2.0)
            axisCom.put(".HOMF", 1)
            wait_for = 3.0
            mip = int(axisCom.get(".MIP"))
            doTheLoop = True
            while doTheLoop:
                mip = int(axisCom.get(".MIP"))
                if mip == axisMr.MIP_BIT_HOMF:
                    doTheLoop = False
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} mip=0x{mip:04x} {axisMr.getMIPtext(mip)} doTheLoop={doTheLoop}"
                )
                if wait_for < 0.0:
                    doTheLoop = False
                if doTheLoop:
                    time.sleep(polltime)
                    wait_for -= polltime

            if wait_for < 0.0:
                testPassed = False
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} wait_for={wait_for:.2f} testPassed={testPassed}"
            )
        if testPassed:
            axisCom.put(".VAL", pos_to_reach)
            time_to_wait = axisMr.calcTimeOut(
                pos_to_reach, velo
            ) + axisMr.calcHomeTimeOut(tc_no)
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} velo={velo:.2f} time_to_wait={time_to_wait:.2f}"
            )
            axisMr.waitForMipZero(tc_no, time_to_wait)

        axisCom.put(".VELO", old_velo)
        axisMr.setFieldSPAM(tc_no, oldSPAM)
        if testPassed:
            self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
        else:
            self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
        assert testPassed

    def teardown_class(self):
        tc_no = int(filnam) * 10100 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
