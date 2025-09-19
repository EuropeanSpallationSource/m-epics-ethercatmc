#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom


filnam = os.path.basename(__file__)[0:3]
###

# How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1

noFRAC = 1.0
withFRAC = 1.5
idxStatusCodeBusyPowerOn = "0x60400000"


def lineno():
    return inspect.currentframe().f_back.f_lineno


def jogAndBacklash(self, tc_no, frac, encRel, maxcnt, StartPos, EndPos, myJOGX):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.axisMr.motorInitAllForBDSTIfNeeded(tc_no)
    self.axisCom.put(".FRAC", frac)
    self.axisCom.put(".UEIP", encRel)
    self.axisCom.put(".RTRY", 1)

    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    self.axisMr.setFieldSPAM(tc_no, -1)
    testPassed = self.axisMr.setMotorStartPos(tc_no, StartPos)

    if not testPassed:
        self.axisCom.putDbgStrToLOG("FailedX " + str(tc_no), wait=True)
    assert testPassed

    self.axisMr.setValueOnSimulator(tc_no, "log", actFileName)
    if myJOGX == "JOGF":
        myDirection = 1
        expMip = self.axisMr.MIP_BIT_JOGF
    elif myJOGX == "JOGR":
        myDirection = 0
        expMip = self.axisMr.MIP_BIT_JOGR
    else:
        assert 0
    self.axisMr.writeExpFileJOG_BDST(
        tc_no,
        expFileName,
        myDirection,
        frac,
        encRel,
        maxcnt,
        StartPos,
        EndPos,
    )
    start_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
    start_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} myJOGX={myJOGX} start_change_cnt_dmov_true={start_change_cnt_dmov_true} start_change_cnt_dmov_false={start_change_cnt_dmov_false}"
    )
    field_name = "." + myJOGX
    # Add the dot between the motorRecord name and the field
    self.axisCom.put(field_name, 1)
    time_to_wait = 1.0
    self.axisMr.waitForValueChanged(
        tc_no, ".MIP", expMip, 0, time_to_wait, debugPrint=True
    )
    self.axisMr.setValueOnSimulator(tc_no, "nStatReasAUX", idxStatusCodeBusyPowerOn)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)
    self.axisMr.setValueOnSimulator(tc_no, "fFastMove", EndPos)
    self.axisCom.put(field_name, 0)  # stop the jogging
    # wait for MR to issue the STOP towards the MCU
    self.axisMr.waitForValueChanged(
        tc_no, ".MIP", self.axisMr.MIP_BIT_JOG_STOP, 0, time_to_wait, debugPrint=True
    )
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    # wait for backlash to be done
    time_to_wait = 500
    self.axisMr.waitForMipZero(tc_no, time_to_wait)
    self.axisMr.setValueOnSimulator(tc_no, "dbgCloseLogFile", "1")

    passed = self.axisMr.cmpUnlinkExpectedActualFile(tc_no, expFileName, actFileName)
    end_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
    end_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
    num_change_cnt_dmov_true = end_change_cnt_dmov_true - start_change_cnt_dmov_true
    num_change_cnt_dmov_false = end_change_cnt_dmov_false - start_change_cnt_dmov_false
    passed = passed and num_change_cnt_dmov_true == 1 and num_change_cnt_dmov_false == 1
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} end_change_cnt_dmov_true={end_change_cnt_dmov_true} end_change_cnt_dmov_false={end_change_cnt_dmov_false}"
    )
    num_change_cnt_dmov_true = end_change_cnt_dmov_true - start_change_cnt_dmov_true
    num_change_cnt_dmov_false = end_change_cnt_dmov_false - start_change_cnt_dmov_false
    passed = passed and num_change_cnt_dmov_true == 1 and num_change_cnt_dmov_false == 1
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    axisCom.put("-DbgStrToLOG", "Start " + os.path.basename(__file__)[0:20])

    myPOSlow = axisMr.myPOSlow
    myPOSmid = axisMr.myPOSmid
    myPOShig = axisMr.myPOShig

    def test_TC_91000(self):
        tc_no = "91000"
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        self.axisMr.initializeMotorRecordSimulatorAxis(tc_no)
        self.axisMr.motorInitAllForBDST(tc_no)
        self.axisMr.setFieldSPAM(tc_no, 255)
        self.axisCom.putDbgStrToLOG("Finish " + str(tc_no), wait=True)

    # JOG forward & backlash compensation, absolute
    def test_TC_91011(self):
        jogAndBacklash(
            self,
            91011,
            noFRAC,
            use_abs,
            1,
            self.myPOSlow,
            self.myPOSmid,
            "JOGF",
        )

    # JOG forward & backlash compensation, relative
    def test_TC_91012(self):
        jogAndBacklash(
            self,
            91012,
            noFRAC,
            use_rel,
            1,
            self.myPOSmid,
            self.myPOSlow,
            "JOGF",
        )

    # JOG backward & backlash compensation, absolute
    def test_TC_91021(self):
        jogAndBacklash(
            self,
            91021,
            noFRAC,
            use_abs,
            1,
            self.myPOSlow,
            self.myPOSmid,
            "JOGR",
        )

    # JOG backward & backlash compensation, relative
    def test_TC_91022(self):
        jogAndBacklash(
            self,
            91022,
            noFRAC,
            use_rel,
            1,
            self.myPOSmid,
            self.myPOSlow,
            "JOGR",
        )

    # JOG forward & backlash compensation, absolute
    def test_TC_91031(self):
        tc_no = 91031
        # motor/master: Uses FRAC for absolute (which is wrong, I think)
        # and ends up in a retry.
        isMotorMaster = self.axisMr.getIsMotorMaster(tc_no)
        if isMotorMaster:
            maxcnt = 1 + int(self.axisCom.get(".RTRY"))
            myFRAC = withFRAC
        else:
            maxcnt = 1
            myFRAC = noFRAC
        jogAndBacklash(
            self,
            tc_no,
            myFRAC,
            use_abs,
            maxcnt,
            self.myPOSlow,
            self.myPOSmid,
            "JOGF",
        )

    # JOG forward & backlash compensation, relative
    def test_TC_91032(self):
        jogAndBacklash(
            self,
            91032,
            noFRAC,  ##withFRAC,
            use_rel,
            1,
            self.myPOSmid,
            self.myPOSlow,
            "JOGF",
        )

    # JOG backward & backlash compensation, absolute
    def test_TC_91041(self):
        maxcnt = 1 + int(self.axisCom.get(".RTRY"))
        tc_no = 91031
        # motor/master: Uses FRAC for absolute (which is wrong, I think)
        # and ends up in a retry.
        isMotorMaster = self.axisMr.getIsMotorMaster(tc_no)
        if isMotorMaster:
            maxcnt = 1 + int(self.axisCom.get(".RTRY"))
            myFRAC = withFRAC
            # Workaround: need to reset the .RCNT field
            self.axisCom.put(".FRAC", 1.0)
            self.axisCom.put(".BDST", 0.0)
            myPos = (self.myPOSlow + self.myPOSmid) / 2.0
            self.axisCom.put(".VAL", myPos, wait=True)
        else:
            maxcnt = 1
            myFRAC = noFRAC
        debug_text = f"{tc_no}#{lineno()} maxcnt={maxcnt} myFRAC={myFRAC}"
        self.axisCom.putDbgStrToLOG(debug_text, wait=True)
        jogAndBacklash(
            self,
            91041,
            myFRAC,
            use_abs,
            maxcnt,
            self.myPOSlow,
            self.myPOSmid,
            "JOGR",
        )

    # JOG backward & backlash compensation, relative
    def test_TC_91042(self):
        jogAndBacklash(
            self,
            91042,
            noFRAC,  ##withFRAC,
            use_rel,
            1,
            self.myPOSmid,
            self.myPOSlow,
            "JOGR",
        )

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
