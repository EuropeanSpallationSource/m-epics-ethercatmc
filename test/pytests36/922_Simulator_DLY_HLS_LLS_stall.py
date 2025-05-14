#!/usr/bin/env python
#


# caput IOC:m1.DLY 1.0
# caput IOC:m1.RTRY
# caput IOC:m1.RMOD
# caput IOC:m1-DbgStrToMCU "Sim.this.nStatReasAUX=1F400000"


import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]
tc_no_base = int(os.path.basename(__file__)[0:3]) * 10
###

motorRMOD_D = 0  # "Default"
use_abs = 0
direction = 1

# https://forge.frm2.tum.de/public/doc/plc/v2.0/singlehtml/
# statusReasonAux is a 32 bit word:
# bit 31..28           1=Idle, 3=Warn, 8=Error
# bit 27               High limit (switch)
# bit 26               Low limit (switch)
# bit 25               Dynamic problem (stall)
# bit 24               Static problem (air pressure, not used here)
# bit 23               not homed (simulator)
# bit 22               enabled (simulator)
# bit 21               (not used in simulator)
# bit 20               local mode (simulator, not used)
# bit 19               InterlockFwd (simulator, not used)
# bit 18               InterlockBwd (simulator, not used)
# bit 17..0            not used

statusReasonAuxIdleEnabled = "0x10400000"
statusReasonAuxIdleStallEnabled = "0x12400000"
statusReasonAuxIdleSLlsHlsEnabled = "0x1C400000"


def lineno():
    return inspect.currentframe().f_back.f_lineno


def positionAndSomeProblemNoRetry(
    self, tc_no, dly, motorStartPos, motorEndPos, statusReasonAux
):
    ###########

    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.axisMr.setValueOnSimulator(tc_no, "fSimForcePos", motorStartPos)
    self.axisCom.put(".STUP", 1)
    maxDelta = 0.1
    timeout = 4.0
    posOk = self.axisMr.waitForValueChanged(
        tc_no, ".RBV", motorStartPos, maxDelta, timeout
    )
    # Create a "expected" file
    # Note that retry should not be expected
    self.axisCom.put(".RTRY", 0)
    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    expFile = open(expFileName, "w")
    self.axisMr.writeExpFileRMOD_X(
        tc_no,
        motorRMOD_D,
        expFile,
        0.0,  # myFRAC,
        0,  # use_abs,
        motorStartPos,
        motorEndPos,
    )
    expFile.close()

    # The motor should be configured to do a retry
    # However, since we run into a problem, the log file will
    # not expect a retry
    self.axisCom.put(".DLY", dly)
    self.axisCom.put(".RTRY", 2)
    self.axisMr.setFieldSPAM(tc_no, -1)

    self.axisMr.setValueOnSimulator(tc_no, "log", actFileName)
    start_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
    start_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} start_change_cnt_dmov_true={start_change_cnt_dmov_true} start_change_cnt_dmov_false={start_change_cnt_dmov_false}"
    )
    self.axisCom.put(".VAL", motorEndPos)
    wait_for_start = 2
    startOk = self.axisMr.waitForStart(tc_no, wait_for_start, throw=False)
    # set the simulated statusReasonAux
    self.axisMr.setValueOnSimulator(tc_no, "nStatReasAUX", statusReasonAux)
    # activate simulated statusReasonAux
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)

    time_to_wait = 20
    stopOk = self.axisMr.waitForStop(tc_no, time_to_wait, throw=False)
    self.axisMr.setValueOnSimulator(tc_no, "dbgCloseLogFile", "1")

    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    fileOk = self.axisMr.cmpUnlinkExpectedActualFile(tc_no, expFileName, actFileName)
    if self.axisMr.hasFieldMFLG:
        # Need to switch of special handling in motorRecord for
        # this very test
        # self.axisCom.put("-LsRampDown", 1, wait=True)
        # self.axisCom.put("-NoStopOnLs", 1, wait=True)
        mflg = int(self.axisCom.get(".MFLG"))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} mflg=0x{mflg:04X}"
        )
    end_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
    end_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
    num_change_cnt_dmov_true = end_change_cnt_dmov_true - start_change_cnt_dmov_true
    num_change_cnt_dmov_false = end_change_cnt_dmov_false - start_change_cnt_dmov_false
    dmovOk = num_change_cnt_dmov_true == 1 and num_change_cnt_dmov_false == 1
    testPassed = posOk and startOk and stopOk and fileOk and dmovOk
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} posOk={posOk} startOk={startOk} stopOk={stopOk} fileOk={fileOk} dmovOk={dmovOk}"
    )
    if not dmovOk:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} end_change_cnt_dmov_true={end_change_cnt_dmov_true} end_change_cnt_dmov_false={end_change_cnt_dmov_false}"
        )
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    return testPassed


class Test(unittest.TestCase):
    tc_no = tc_no_base
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )
    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    hasFieldMFLG = axisMr.hasFieldMFLG
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} hasFieldMFLG={hasFieldMFLG}"
    )
    if hasFieldMFLG:
        # Need to switch of special handling in motorRecord for
        # this very test
        axisCom.put("-LsRampDown", 0, wait=True)
        axisCom.put("-NoStopOnLs", 0, wait=True)
        mflg = int(axisCom.get(".MFLG"))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} mflg=0x{mflg:04X}"
        )

    myPOSlow = axisMr.myPOSlow
    myPOShig = axisMr.myPOShig
    saved_DLY = axisCom.get(".DLY")
    saved_RMOD = axisCom.get(".RMOD")
    saved_RTRY = axisCom.get(".RTRY")
    saved_VELO = axisCom.get(".VELO")
    axisMr.motorInitAllForBDSTIfNeeded(tc_no)
    axisCom.put(".RMOD", motorRMOD_D)
    axisCom.put(".UEIP", 0)
    axisCom.put(".RTRY", 0)
    axisCom.put(".FRAC", 0.0)
    axisCom.put(".BDST", 0.0)
    axisCom.put(".VELO", 1.0)

    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}")

    # motorRMOD_D = 0 # "Default"
    # position forward BDST == 0 RTRY == 0
    # Checks for a regression in motorRecord.cc@ess-master
    # Which may cause a never ending loop
    def test_TC_922001(self):
        dly = 1.0
        assert positionAndSomeProblemNoRetry(
            self,
            922001,
            dly,
            self.myPOSlow,
            self.myPOShig,
            statusReasonAuxIdleSLlsHlsEnabled,
        )

    def test_TC_922002(self):
        dly = 0.0
        assert positionAndSomeProblemNoRetry(
            self,
            922002,
            dly,
            self.myPOSlow,
            self.myPOShig,
            statusReasonAuxIdleSLlsHlsEnabled,
        )

    def test_TC_922003(self):
        dly = 1.0
        assert positionAndSomeProblemNoRetry(
            self,
            922003,
            dly,
            self.myPOSlow,
            self.myPOShig,
            statusReasonAuxIdleStallEnabled,
        )

    def test_TC_922004(self):
        dly = 0.0
        assert positionAndSomeProblemNoRetry(
            self,
            922004,
            dly,
            self.myPOSlow,
            self.myPOShig,
            statusReasonAuxIdleStallEnabled,
        )

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        if self.hasFieldMFLG:
            self.axisCom.put("-LsRampDown", 1, wait=True)
            self.axisCom.put("-NoStopOnLs", 1, wait=True)

        self.axisCom.put(".DLY", self.saved_DLY)
        self.axisCom.put(".RMOD", self.saved_RMOD)
        self.axisCom.put(".RTRY", self.saved_RTRY)
        self.axisCom.put(".VELO", self.saved_VELO)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
