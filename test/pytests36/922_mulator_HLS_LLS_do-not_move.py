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


def lineno():
    return inspect.currentframe().f_back.f_lineno


polltime = 0.1

# Values to be used for test
# Note: Make sure to use different values to hae a good
# test coverage
myVELO = 20.0  # positioning velocity
myACCL = 1.0  # Time to VELO, seconds

# Different values, high use even, low uses odd
#
myLowHardLimitPosVAL = -19.0
myDLLM = 0.0
myStartposDial = 0.0
myDHLM = 0.0
myHighHardLimitPosVAL = 18.0

# Comparing floating points may fail because of rounding problems
maxdelta = 0.01


def InitAllFor922(self, tc_no):
    # msta = int(self.axisCom.get(".MSTA"))
    # assert msta & self.axisMr.MSTA_BIT_HOMED  # , 'MSTA.homed (Axis has been homed)')

    self.axisCom.put(".MRES", 1.0)
    self.axisCom.put(".DIR", 0)
    self.axisCom.put(".VELO", myVELO)
    self.axisCom.put(".ACCL", myACCL)
    self.axisCom.put(".SPDB", 0.1)
    self.axisCom.put(".RDBD", 0.1)
    self.axisCom.put(".BDST", 0.0)
    self.axisMr.setValueOnSimulator(tc_no, "bAxisHomed", 1)

    self.axisMr.waitForStop(tc_no, 2.0)
    self.axisMr.setValueOnSimulator(tc_no, "fSimForcePos", 0.0)
    self.axisMr.setValueOnSimulator(tc_no, "bAxisHomed", 1)
    self.axisMr.setValueOnSimulator(tc_no, "nAmplifierPercent", 100)
    self.axisMr.setValueOnSimulator(
        tc_no, "fHighHardLimitPos", self.axisMr.VALtoRVAL(tc_no, myHighHardLimitPosVAL)
    )
    self.axisMr.setValueOnSimulator(
        tc_no, "fHighSoftLimitPos", self.axisMr.VALtoRVAL(tc_no, myHighHardLimitPosVAL)
    )
    self.axisMr.setValueOnSimulator(
        tc_no, "fLowHardLimitPos", self.axisMr.VALtoRVAL(tc_no, myLowHardLimitPosVAL)
    )
    self.axisMr.setValueOnSimulator(
        tc_no, "fLowSoftLimitPos", self.axisMr.VALtoRVAL(tc_no, myLowHardLimitPosVAL)
    )
    self.axisMr.setSoftLimitsOff(tc_no)
    isMotorMaster = self.axisMr.getIsMotorMaster(tc_no)
    if isMotorMaster:
        self.axisCom.put("-PwrAuto", 0)
    self.axisMr.powerOnHomeAxis(tc_no)


def writeExpFileDontMoveThenMoveWhenOnLS(
    self, tc_no, expFileName, startPosRVAL, endPosVAL
):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: startPosRVAL={startPosRVAL:.2f} endPosVAL={endPosVAL:.2f} "
    )
    debug_text = f"{tc_no} Strt={startPosRVAL} End={endPosVAL}"
    self.axisCom.putDbgStrToLOG(debug_text, wait=True)
    velo = float(self.axisCom.get(".VELO"))
    accs = float(self.axisCom.get(".ACCS"))  # accelation in EGU/sec/sec
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: velo={velo:.2f} accs={accs:.2f}"
    )
    # Create a "expected" file
    expFile = open(expFileName, "w")

    # We should move away from the LS
    line1 = (
        "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
        % (endPosVAL, velo, accs, startPosRVAL)
    )
    expFile.write(f"{line1}")
    expFile.write("EOF\n")
    expFile.close()


def moveIntoLimitSwitchCheckMoveOrNotOneField(
    self, tc_no, lsToBeActiveted="", mres=0, dirPlusMinus=0, field="", value=""
):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsToBeActiveted={lsToBeActiveted} mres={mres} dirPlusMinus={dirPlusMinus} field={field} value={value}"
    )
    self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
    assert dirPlusMinus != 0
    assert mres != 0
    oldMres = float(self.axisCom.get(".MRES"))
    if mres != oldMres:
        self.axisCom.put(".MRES", mres)

    if dirPlusMinus == 1:
        dir = 0  # motorRecords .DIR is 0 for "normal", 1 for "invers"
    elif dirPlusMinus == -1:
        dir = 1
    else:
        assert False
    oldDir = int(self.axisCom.get(".DIR"))
    if oldDir != dir or mres != oldMres:
        # Need to go to 0.0, before DIR can be reverted
        self.axisMr.setValueOnSimulator(tc_no, "fSimForcePos", 0.0)
        self.axisMr.moveWait(tc_no, 0.0)
        self.axisCom.put(".DIR", dir)
        self.axisCom.put(".OFF", 0.0)

    twv = float(self.axisCom.get(".TWV"))
    if lsToBeActiveted == "LLS":
        softlimitPositionVAL = myLowHardLimitPosVAL
        endPosVAL = myLowHardLimitPosVAL + twv
    elif lsToBeActiveted == "HLS":
        softlimitPositionVAL = myHighHardLimitPosVAL
        endPosVAL = myHighHardLimitPosVAL - twv
    else:
        assert False
    softlimitPositionRAW = self.axisMr.VALtoRVAL(tc_no, softlimitPositionVAL)
    endPosRAW = self.axisMr.VALtoRVAL(tc_no, endPosVAL)
    lsActive = int(self.axisCom.get("." + lsToBeActiveted))
    if lsActive != 0:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} softlimitPositionVAL={softlimitPositionVAL} endPosVAL={endPosVAL} softlimitPositionRAW={softlimitPositionRAW}"
        )
        self.axisMr.setValueOnSimulator(tc_no, "fSimForcePos", softlimitPositionRAW)
        self.axisMr.doSTUPandSYNC(tc_no)
    maxDelta = 0.1
    timeout = 3.0
    valueVALok = self.axisMr.waitForValueChanged(
        tc_no, ".RBV", softlimitPositionVAL, maxDelta, timeout
    )
    lsActive = int(self.axisCom.get("." + lsToBeActiveted))
    hls = int(self.axisCom.get(".HLS"))
    lls = int(self.axisCom.get(".LLS"))
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsToBeActiveted={lsToBeActiveted} lsActive={lsActive} lls={lls} hls={hls} twv={twv} endPosVAL={endPosVAL}"
    )
    passed = lsActive
    ##
    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    writeExpFileDontMoveThenMoveWhenOnLS(
        self, tc_no, expFileName, softlimitPositionRAW, endPosRAW
    )
    self.axisMr.setValueOnSimulator(tc_no, "log", actFileName)
    # try to move beyond the limit switch, then away from it
    # Only the second movement should be commanded
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} field={field} value={value}"
    )
    # self.axisCom.put(".DVAL", endPos, wait=True)
    if field == ".VAL" or field == ".DVAL":
        self.axisMr.moveWait(tc_no, value, field=field)
    elif field == ".TWF" or field == ".TWR" or field == ".JOGF" or field == ".JOGR":
        self.axisCom.put(field, value, wait=True, timeout=60)
    else:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} illegal field={field}"
        )
        assert False

    self.axisMr.moveWait(tc_no, endPosVAL, field=".VAL")

    self.axisMr.setValueOnSimulator(tc_no, "dbgCloseLogFile", "1")
    fileCmpOk = self.axisMr.cmpUnlinkExpectedActualFile(tc_no, expFileName, actFileName)
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsActive={lsActive} valueVALok={valueVALok} fileCmpOk={fileCmpOk}"
    )
    passed = valueVALok and fileCmpOk and lsActive == 1
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


def moveIntoLimitSwitchCheckMoveOrNotWrapper(
    self, tc_no, lsToBeActiveted="", mres=0, dirPlusMinus=0
):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsToBeActiveted={lsToBeActiveted} mres{mres} dirPlusMinus={dirPlusMinus}"
    )
    assert mres != 0
    assert dirPlusMinus != 0
    twv = float(self.axisCom.get(".TWV"))
    # if (mres * dirPlusMinus) > 0:
    if lsToBeActiveted == "LLS":
        beyondLimitSwitchPosVAL = myLowHardLimitPosVAL - twv
        rf = "R"
    elif lsToBeActiveted == "HLS":
        beyondLimitSwitchPosVAL = myLowHardLimitPosVAL + twv
        rf = "F"
    else:
        assert False
    beyondLimitSwitchPosDVAL = beyondLimitSwitchPosVAL * dirPlusMinus
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} beyondLimitSwitchPosVAL={beyondLimitSwitchPosVAL} beyondLimitSwitchPosDVAL={beyondLimitSwitchPosDVAL} TW{rf} JOG{rf}"
    )
    # creating a dictionary
    field_value_to_be_tested = {
        # ".DVAL": beyondLimitSwitchPosVAL,
        # ".VAL": beyondLimitSwitchPosVAL * dirPlusMinus,
        # ".TW" + rf: 1,
        ".JOG"
        + rf: 1,
    }
    counter = 0
    for field in field_value_to_be_tested:
        counter = counter + 1
        value = field_value_to_be_tested[field]
        moveIntoLimitSwitchCheckMoveOrNotOneField(
            self,
            tc_no + counter,
            lsToBeActiveted=lsToBeActiveted,
            mres=mres,
            dirPlusMinus=dirPlusMinus,
            field=field,
            value=value,
        )


class Test(unittest.TestCase):
    drvUseEGU_RB = None
    drvUseEGU = 0
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    # Initialize
    def test_TC_9220000(self):
        tc_no = 9220000
        self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
        self.axisMr.setFieldSPAM(tc_no, 2047)

        InitAllFor922(self, tc_no)
        self.axisCom.putDbgStrToLOG("End " + str(tc_no), wait=True)

    def test_TC_9221000(self):
        tc_no = 9221000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="LLS", mres=1.0, dirPlusMinus=1
        )

    def test_TC_9222000(self):
        tc_no = 9220000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="LLS", mres=1.0, dirPlusMinus=-1
        )

    def XXXX_9223000(self):
        tc_no = 9223000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="LLS", mres=-1.0, dirPlusMinus=-1
        )

    def XXXX_9224000(self):
        tc_no = 9224000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="LLS", mres=-1.0, dirPlusMinus=1
        )

    def XXXX_9225000(self):
        tc_no = 9225000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="HLS", mres=1.0, dirPlusMinus=-1
        )

    def XXXX_9226000(self):
        tc_no = 9226000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="HLS", mres=1.0, dirPlusMinus=1
        )

    def XXXX_9227000(self):
        tc_no = 9227000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="HLS", mres=-1.0, dirPlusMinus=-1
        )

    def XXXX_9228000(self):
        tc_no = 9228000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapper(
            self, tc_no, lsToBeActiveted="HLS", mres=-1.0, dirPlusMinus=1
        )

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
