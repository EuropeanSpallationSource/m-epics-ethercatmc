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
myJVEL = 20.0  # jogging velocity
myACCS = 1.0  # Time to BVEL, seconds
myACCL = 1.0  # Time to VELO, seconds


# Different values, high use even, low uses odd
#
myLowHardLimitPosVAL = -21.0
myDLLM = 0.0
myStartposDial = 0.0
myDHLM = 0.0
myHighHardLimitPosVAL = 18.0

# Comparing floating points may fail because of rounding problems
maxDelta = 0.01


def InitAllFor921(self, tc_no):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: self.initAllFor921Done={self.initAllFor921Done}"
    )
    if self.initAllFor921Done is True:
        return
    self.axisMr.setValueOnSimulator(tc_no, "nAmplifierPercent", 0)
    self.axisMr.setValueOnSimulator(tc_no, "setMRES_23", 0)
    self.axisMr.setValueOnSimulator(tc_no, "setMRES_24", 0)
    drvUseEGU_RB = self.axisCom.get("-DrvUseEGU-RB")
    if drvUseEGU_RB == 1:
        self.axisCom.put("-DrvUseEGU", 0)
    if self.axisMr.hasFieldMFLG:
        mflg = int(self.axisCom.get(".MFLG"))
        if mflg & self.axisMr.MF_DRIVER_USES_EGU:
            time_to_wait = 3.0
            # set bit to 0 using xor
            mflgNew = mflg ^ self.axisMr.MF_DRIVER_USES_EGU
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} mflg=0x{mflg:04X} mflgNew=0x{mflgNew:04X}"
            )
            self.axisMr.waitForValueChangedInt32(tc_no, ".MFLG", mflgNew, time_to_wait)
    self.axisCom.put(".VELO", myVELO)
    self.axisCom.put(".BVEL", myVELO / 2.0)
    self.axisCom.put(".JVEL", myJVEL)
    self.axisCom.put(".ACCL", myACCL)
    self.axisCom.put(".ACCS", myACCS)
    self.axisCom.put(".BACC", myACCS)
    self.axisCom.put(".SPDB", 0.1)
    self.axisCom.put(".RDBD", 0.1)
    self.axisCom.put(".BDST", 0.0)
    self.axisMr.setValueOnSimulator(tc_no, "fSimForcePos", 0.0)
    self.axisMr.setValueOnSimulator(tc_no, "bAxisHomed", 1)
    self.axisMr.setValueOnSimulator(tc_no, "nAmplifierPercent", 100)
    # self.axisMr.waitForStop(tc_no, 2.0)
    self.axisMr.setValueOnSimulator(
        tc_no,
        "fHighHardLimitPos",
        myHighHardLimitPosVAL,
    )
    self.axisMr.setValueOnSimulator(
        tc_no,
        "fHighSoftLimitPos",
        myHighHardLimitPosVAL,
    )
    self.axisMr.setValueOnSimulator(
        tc_no,
        "fLowHardLimitPos",
        myLowHardLimitPosVAL,
    )
    self.axisMr.setValueOnSimulator(
        tc_no,
        "fLowSoftLimitPos",
        myLowHardLimitPosVAL,
    )
    self.axisMr.setSoftLimitsOff(tc_no)
    isMotorMaster = self.axisMr.getIsMotorMaster(tc_no)
    if isMotorMaster:
        self.axisCom.put("-PwrAuto", 0)
    self.axisMr.powerOnHomeAxis(tc_no)
    self.initAllFor921Done = True
    self.axisMr.setFieldSPAM(tc_no, 2047)


def writeExpFileDontMoveThenMoveWhenOnLS(
    self, tc_no, expFileName, startPosRAW, endPosRAW
):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} startPosRAW={startPosRAW:.2f} endPosRAW={endPosRAW:.2f}"
    )
    debug_text = f"{tc_no} Strt={startPosRAW} End={endPosRAW}"
    self.axisCom.putDbgStrToLOG(debug_text, wait=True)
    velo = float(self.axisCom.get(".VELO"))
    accs = float(self.axisCom.get(".ACCS"))  # accelation in EGU/sec/sec
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: velo={velo:.2f} accs={accs:.2f}"
    )
    # Create a "expected" file
    expFile = open(expFileName, "w")

    # We should move away from the LS
    bdst = float(self.axisCom.get(".BDST"))
    mres = float(self.axisCom.get(".MRES"))
    if mres < 0.0:
        bdstRAW = 0.0 - bdst
    else:
        bdstRAW = bdst
    diff = endPosRAW - startPosRAW
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: bdst={bdst:.2f} bdstRAW={bdstRAW:.2f} diff={diff:.2f}"
    )
    if bdstRAW == 0:
        line1 = (
            "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
            % (endPosRAW, velo, accs, startPosRAW)
        )
        expFile.write(f"{line1}")
    else:
        bvel = float(self.axisCom.get(".BVEL"))
        bacc = float(self.axisCom.get(".BACC"))
        if bacc > 0:
            bccs = bvel / bacc
        else:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: bacc==0.0"
            )
            assert False
        # We do not use "short moves"
        # if diff * bdst > 0 and fabs(diff) < fabs(bdst):
        #    # Same direction, short move
        #    line2 =
        intermediatePositionRAW = endPosRAW - bdstRAW
        line2 = (
            "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
            % (intermediatePositionRAW, velo, accs, startPosRAW)
        )
        line3 = (
            "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
            % (endPosRAW, bvel, bccs, intermediatePositionRAW)
        )
        expFile.write(f"{line2}{line3}")
    expFile.write("EOF\n")
    expFile.close()


def moveIntoLimitSwitchCheckMoveOrNotOneField(
    self,
    tc_no,
    lsToBeActiveted="",
    mres=0,
    dirFieldPlusMinus=0,
    field="",
    value="",
    bdst="",
):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsToBeActiveted={lsToBeActiveted} mres={mres} dirFieldPlusMinus={dirFieldPlusMinus} field={field} value={value} bdst={bdst}"
    )
    self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
    assert dirFieldPlusMinus != 0
    assert mres != 0
    assert bdst != ""
    if dirFieldPlusMinus == 1:
        dir = 0  # motorRecords .DIR is 0 for "normal", 1 for "invers"
    elif dirFieldPlusMinus == -1:
        dir = 1
    else:
        assert False
    oldDir = int(self.axisCom.get(".DIR"))
    oldMres = float(self.axisCom.get(".MRES"))
    dirOrMresChanged = oldDir != dir or mres != oldMres
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} oldDir={oldDir} dir={dir} mres={mres} oldMres={oldMres} dirOrMresChanged={dirOrMresChanged}"
    )
    while dirOrMresChanged:
        # Need to go to 0.0, before DIR can be reverted
        # self.axisMr.setValueOnSimulator(tc_no, "fSimForcePos", 0.0)
        self.axisCom.put(".OFF", 0.0)
        self.axisMr.moveWait(tc_no, 0.0)
        if oldDir != dir:
            self.axisCom.put(".DIR", dir)
        if mres != oldMres:
            self.axisCom.put(".MRES", mres)
        self.axisCom.put(".OFF", 0.0)
        self.axisMr.setValueOnSimulator(tc_no, "fSimForcePos", 0.0)
        rbv = float(self.axisCom.get(".RBV"))
        drbv = float(self.axisCom.get(".DRBV"))
        mip = int(self.axisCom.get(".DRBV"))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} rbv={rbv:.2f} drbv={drbv:.2f} mip={self.axisMr.getMIPtext(mip)}"
        )
        oldDir = int(self.axisCom.get(".DIR"))
        oldMres = float(self.axisCom.get(".MRES"))
        if rbv == 0.0 and drbv == 0.0 and mip == 0:
            dirOrMresChanged = False

    twv = float(self.axisCom.get(".TWV"))
    if lsToBeActiveted == "LLS":
        jogTowardsLimitSwitch = ".JOGR"
    elif lsToBeActiveted == "HLS":
        jogTowardsLimitSwitch = ".JOGF"
    else:
        assert False
    lsActive = int(self.axisCom.get("." + lsToBeActiveted))
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsToBeActiveted={lsToBeActiveted} lsActive={lsActive}"
    )
    timeout = (myHighHardLimitPosVAL - myLowHardLimitPosVAL) / myJVEL + 5.0
    if lsActive != 1:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} jogTowardsLimitSwitch={jogTowardsLimitSwitch}"
        )
        self.axisCom.put(".BDST", 0.0)
        self.axisCom.put(jogTowardsLimitSwitch, 1)
        self.axisMr.waitForValueChanged(
            tc_no, "." + lsToBeActiveted, 1, maxDelta, timeout
        )
        self.axisMr.waitForStop(tc_no, 2.0)
    self.axisCom.put(".BDST", bdst)
    lsActive = int(self.axisCom.get("." + lsToBeActiveted))
    hls = int(self.axisCom.get(".HLS"))
    lls = int(self.axisCom.get(".LLS"))
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsToBeActiveted={lsToBeActiveted} lsActive={lsActive} lls={lls} hls={hls}"
    )
    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"
    rbv = float(self.axisCom.get(".RBV"))
    rrbv = self.axisMr.calcRVALfromVAL(tc_no, rbv)
    if lls:
        soflimitNotActivatedVAL = rbv + 2 * twv
        soflimitActivatedVAL = rbv - 2 * twv
    elif hls:
        soflimitNotActivatedVAL = rbv - 2 * twv
        soflimitActivatedVAL = rbv + 2 * twv
    else:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} illegal lls={lls} hls={hls}"
        )
        assert False
    soflimitNotActivatedRAW = self.axisMr.calcRVALfromVAL(
        tc_no, soflimitNotActivatedVAL
    )
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} field={field} rbv={rbv:.2f} rrbv={rrbv:.2f}, soflimitNotActivated USER={soflimitNotActivatedVAL:.2f} RAW={soflimitNotActivatedRAW:.2f}"
    )

    writeExpFileDontMoveThenMoveWhenOnLS(
        self, tc_no, expFileName, rrbv, soflimitNotActivatedRAW
    )
    self.axisMr.setValueOnSimulator(tc_no, "log", actFileName)
    # try to move beyond the limit switch, then away from it
    # Only the second movement should be commanded
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} field={field} value={value}"
    )
    had_ex = False
    try:
        # first movement: This should be suppressed, since limit switch actived
        if field == ".VAL":
            # Change value if MRES < 0
            value = soflimitActivatedVAL
            self.axisMr.moveWait(tc_no, value)
        elif field == ".TWF" or field == ".TWR" or field == ".JOGF" or field == ".JOGR":
            self.axisCom.put(field, value, wait=True, timeout=60)
        else:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} illegal field={field}"
            )
            assert False
        # second movement: Move away from limit switch
        # only this movement should be found in the log
        self.axisMr.moveWait(tc_no, soflimitNotActivatedVAL)

    except:  # noqa: E722
        had_ex = True
    self.axisMr.setValueOnSimulator(tc_no, "dbgCloseLogFile", "1")
    fileCmpOk = self.axisMr.cmpUnlinkExpectedActualFile(tc_no, expFileName, actFileName)
    passed = not had_ex and fileCmpOk and lsActive == 1
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} had_ex={had_ex} lsActive={lsActive} fileCmpOk={fileCmpOk}"
    )
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} lsToBeActiveted={lsToBeActiveted} mres={mres} dirFieldPlusMinus={dirFieldPlusMinus} field={field} value={value} bdst={bdst} passed={passed}"
    )

    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


def moveIntoLimitSwitchCheckMoveOrNotWrapperAllFields(
    self, tc_no, mres=0, dirFieldPlusMinus=0, lsToBeActiveted="", bdst=""
):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} mres{mres} dirFieldPlusMinus={dirFieldPlusMinus} lsToBeActiveted={lsToBeActiveted}"
    )
    assert mres != 0
    assert dirFieldPlusMinus != 0
    if lsToBeActiveted == "LLS":
        rf = "R"
        beyondLimitSwitchPosVAL = myLowHardLimitPosVAL - 2.0
    elif lsToBeActiveted == "HLS":
        rf = "F"
        beyondLimitSwitchPosVAL = myHighHardLimitPosVAL + 2.0
    else:
        assert False
    # creating a dictionary
    field_value_to_be_tested = {
        ".JOG" + rf: 1,
    }
    if not self.axisMr.isMotorMasterAxis:
        # Add VAL and tweak
        field_value_to_be_tested[".VAL"] = beyondLimitSwitchPosVAL
        field_value_to_be_tested[".TW" + rf] = 1

    counter = 0
    for field in field_value_to_be_tested:
        value = field_value_to_be_tested[field]
        moveIntoLimitSwitchCheckMoveOrNotOneField(
            self,
            tc_no + counter,
            lsToBeActiveted=lsToBeActiveted,
            mres=mres,
            dirFieldPlusMinus=dirFieldPlusMinus,
            field=field,
            value=value,
            bdst=bdst,
        )
        counter = counter + 1


def moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
    self, tc_no, mres=0, dirFieldPlusMinus=0, lsToBeActiveted=""
):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} mres{mres} dirFieldPlusMinus={dirFieldPlusMinus} lsToBeActiveted={lsToBeActiveted}"
    )
    assert mres != 0
    assert dirFieldPlusMinus != 0
    InitAllFor921(self, tc_no)
    counter = 0
    twv = float(self.axisCom.get(".TWV"))
    # Avoid testing BDST in the master axis for now
    if self.axisMr.isMotorMasterAxis:
        BDSTs = [0.0]
    else:
        BDSTs = [0.0, twv / 2.0, -twv / 2.0]
        BDSTs = [0.0]
    for bdst in BDSTs:
        counter = counter + 10
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no+counter} BDST={bdst}"
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperAllFields(
            self,
            tc_no + counter,
            lsToBeActiveted=lsToBeActiveted,
            mres=mres,
            dirFieldPlusMinus=dirFieldPlusMinus,
            bdst=bdst,
        )


class Test(unittest.TestCase):
    drvUseEGU_RB = None
    drvUseEGU = 0
    initAllFor921Done = False
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_921010000(self):
        tc_no = 921010000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=1.0, dirFieldPlusMinus=1, lsToBeActiveted="LLS"
        )

    def test_TC_921020000(self):
        tc_no = 921020000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=1.0, dirFieldPlusMinus=1, lsToBeActiveted="HLS"
        )

    def test_TC_921030000(self):
        tc_no = 921030000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=1.0, dirFieldPlusMinus=-1, lsToBeActiveted="LLS"
        )

    def test_TC_921040000(self):
        tc_no = 921040000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=1.0, dirFieldPlusMinus=-1, lsToBeActiveted="HLS"
        )

    def test_TC_921050000(self):
        tc_no = 921050000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=-1.0, dirFieldPlusMinus=1, lsToBeActiveted="LLS"
        )

    def test_TC_921060000(self):
        tc_no = 921060000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=-1.0, dirFieldPlusMinus=1, lsToBeActiveted="HLS"
        )

    def test_TC_921070000(self):
        tc_no = 921070000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=-1.0, dirFieldPlusMinus=-1, lsToBeActiveted="LLS"
        )

    def test_TC_921080000(self):
        tc_no = 921080000
        self.assertEqual(
            0,
            int(self.axisCom.get(".MSTA")) & self.axisMr.MSTA_BIT_PROBLEM,
            "MSTA.Problem should be 0",
        )
        moveIntoLimitSwitchCheckMoveOrNotWrapperBDST(
            self, tc_no, mres=-1.0, dirFieldPlusMinus=-1, lsToBeActiveted="HLS"
        )

    def teardown_class(self):
        tc_no = int(filnam) * 100000 + 9999
        self.axisCom.put(".BDST", 0.0)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
