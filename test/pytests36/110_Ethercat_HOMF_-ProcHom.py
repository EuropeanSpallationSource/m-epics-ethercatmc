#!/usr/bin/env python

#
# Test differen homing procedures
#


import datetime
import inspect
import unittest
import os
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]

START_FROM_LLS = -1
START_FROM_MID = 0
START_FROM_HLS = 1

HOME_VIA_RECORD = 0  # home via the .HOMF or .HOMR field of the motorRecord
HOME_VIA_DRIVER = 0  # home via the -Home PV talking to the model 3 driver


def lineno():
    return inspect.currentframe().f_back.f_lineno


def homeTheMotor(self, tc_no, homProc, jogToLSBefore, homeViaDriver):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} homProc={homProc} jogToLSBefore={jogToLSBefore} homeViaDriver={homeViaDriver}"
    )
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    old_high_limit = self.axisCom.get(".HLM")
    old_low_limit = self.axisCom.get(".LLM")
    old_HomProc = 0  # default
    old_HomPos = 0.0  # default
    testPassed = True

    if jogToLSBefore != 0:
        msta = int(self.axisCom.get(".MSTA"))
        if msta & self.axisMr.MSTA_BIT_HOMED and old_high_limit > old_low_limit:
            # if we are homed, move absolute to the soft limit
            # This is faster than jogging
            if jogToLSBefore > 0:
                testPassed = testPassed and self.axisMr.moveWait(
                    tc_no, old_high_limit, throw=False
                )
            else:
                testPassed = testPassed and self.axisMr.moveWait(
                    tc_no, old_low_limit, throw=False
                )
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} testPassed={testPassed}"
            )
        self.axisMr.setSoftLimitsOff(tc_no)
        # soft limit range assumed to be = hard range /1.5 or so
        # It is only needed to calculate a good timeout
        jvel = self.axisCom.get(".JVEL")
        accl = self.axisCom.get(".ACCL")
        time_to_wait = 1.5 * (old_high_limit - old_low_limit) / jvel + 2 * accl

        testPassed = testPassed and self.axisMr.jogDirectionTimeout(
            tc_no, jogToLSBefore, time_to_wait
        )
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} testPassed={testPassed}"
        )
        self.axisCom.put(".LLM", old_low_limit)
        self.axisCom.put(".HLM", old_high_limit)
    else:
        testPassed = testPassed and self.axisMr.moveWait(
            tc_no, (old_high_limit + old_low_limit) / 2.0, throw=False
        )
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} testPassed={testPassed}"
        )

    if homProc != 0:
        old_HomProc = self.axisCom.get("-HomProc")
        old_HomPos = self.axisCom.get("-HomPos")

        self.axisCom.put("-HomProc", homProc, wait=True)
        maxcnt = 5
        cnt = 1
        polltime = 0.2
        while cnt < maxcnt:
            homProcRB = int(self.axisCom.get("-HomProc-RB", use_monitor=False))
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} homProc={homProc} homProcRB={homProcRB} cnt={cnt}"
            )
            if homProc != homProcRB:
                time.sleep(polltime)
                cnt += cnt
            else:
                cnt = maxcnt
        self.assertEqual(homProc, homProcRB, tc_no + " homProcRB must be homProc")

        if homProc == 1:
            self.axisCom.put("-HomPos", old_low_limit - 1.0)
        elif homProc == 2:
            self.axisCom.put("-HomPos", old_high_limit + 1.0)

    # Calculate the timeout, based on the driving range
    range_postion = self.axisCom.get(".HLM") - self.axisCom.get(".LLM")
    hvel = self.axisCom.get(".HVEL")
    accl = self.axisCom.get(".ACCL")

    if range_postion > 0 and hvel > 0:
        time_to_wait = 1 + 2 * range_postion / hvel + 2 * accl
    else:
        time_to_wait = 180

    msta = int(self.axisCom.get(".MSTA"))
    # We can home while sitting on a limit switch
    if homeViaDriver == 1:
        self.axisCom.put("-Home", 0)
    elif msta & self.axisMr.MSTA_BIT_MINUS_LS:
        self.axisCom.put(".HOMR", 1)
    else:
        self.axisCom.put(".HOMF", 1)

    try:
        self.axisMr.waitForStart(tc_no, 3)
        msta1 = int(self.axisCom.get(".MSTA"))
        if msta1 & self.axisMr.MSTA_BIT_HOMED:
            unhomed = 0
        else:
            unhomed = 1
            print(
                "%s homeTheMotor msta1=%s unhomed=%d"
                % (tc_no, self.axisMr.getMSTAtext(msta1), unhomed)
            )
    except Exception as ex:
        print(f"{tc_no} waitFoeStart ex={ex} ")

    self.axisMr.waitForStop(tc_no, time_to_wait)

    if homProc != 0:
        self.axisCom.put("-HomProc", old_HomProc)
        self.axisCom.put("-HomPos", old_HomPos)

    msta2 = int(self.axisCom.get(".MSTA"))
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} msta2=={self.axisMr.getMSTAtext(msta2)}"
    )
    if msta2 & self.axisMr.MSTA_BIT_SLIP_STALL:
        testPassed = False
    if not msta2 & self.axisMr.MSTA_BIT_HOMED:
        testPassed = False
    if msta2 & self.axisMr.MSTA_BIT_PROBLEM:
        testPassed = False
        errId = int(self.axisCom.get("-ErrId", use_monitor=False))
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} errId={errId:x}")
        self.axisMr.resetAxis(tc_no)

    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    HomeVis = int(axisCom.get("-HomeVis"))
    homProcRB = int(axisCom.get("-HomProc-RB"))
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} HomeVis={HomeVis} homProcRB={homProcRB}"
    )

    def test_TC_111000(self):
        tc_no = "111000"
        if self.HomeVis == 1:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
            )
            homeTheMotor(self, tc_no, 0, START_FROM_LLS, HOME_VIA_RECORD)

    def test_TC_111001(self):
        tc_no = "111001"
        if self.HomeVis == 1:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
            )
            homeTheMotor(self, tc_no, 0, START_FROM_LLS, HOME_VIA_DRIVER)

    def test_TC_111010(self):
        tc_no = "111010"
        if self.HomeVis == 1:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
            )
            homeTheMotor(self, tc_no, 0, START_FROM_MID, HOME_VIA_RECORD)

    def test_TC_111011(self):
        tc_no = "111011"
        if self.HomeVis == 1:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
            )
            homeTheMotor(self, tc_no, 0, START_FROM_MID, HOME_VIA_DRIVER)

    def test_TC_111020(self):
        tc_no = "111120"
        if self.HomeVis == 1:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
            )
            homeTheMotor(self, tc_no, 0, START_FROM_HLS, HOME_VIA_RECORD)

    def test_TC_111021(self):
        tc_no = "111121"
        if self.HomeVis == 1:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
            )
            homeTheMotor(self, tc_no, 0, START_FROM_HLS, HOME_VIA_DRIVER)

    #    def test_TC_11110(self):
    #        tc_no = "11110"
    #        if self.HomeVis == 1:
    #            print(
    #                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
    #            )
    #            homeTheMotor(self, tc_no, 1, START_FROM_LLS)
    #
    #    def test_TC_11111(self):
    #        tc_no = "11111"
    #        if self.HomeVis == 1:
    #            print(
    #                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
    #            )
    #            homeTheMotor(self, tc_no, 1, START_FROM_MID)
    #
    #    def test_TC_11112(self):
    #        tc_no = "11112"
    #        if self.HomeVis == 1:
    #            print(
    #                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
    #            )
    #            homeTheMotor(self, tc_no, 1, START_FROM_HLS)
    #
    #    def test_TC_11120(self):
    #        tc_no = "11120"
    #        if self.HomeVis == 1:
    #            print(
    #                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
    #            )
    #            homeTheMotor(self, tc_no, 2, START_FROM_LLS)
    #
    #    def test_TC_11121(self):
    #        tc_no = "11121"
    #        if self.HomeVis == 1:
    #            print(
    #                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
    #            )
    #            homeTheMotor(self, tc_no, 2, START_FROM_MID)
    #
    #    def test_TC_11122(self):
    #        tc_no = "11122"
    #        if self.HomeVis == 1:
    #            print(
    #                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} Home the motor"
    #            )
    #            homeTheMotor(self, tc_no, 2, START_FROM_HLS)
    #
    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
