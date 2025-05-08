#!/usr/bin/python

"""

"""

import datetime
import math
import time
import os
import filecmp
import inspect


filnam = "AxisMr"


motorRMOD_D = 0  # "Default"
motorRMOD_A = 1  # "Arithmetic"
motorRMOD_G = 2  # "Geometric"
motorRMOD_I = 3  # "In-Position"

polltime = 0.2


def lineno():
    return inspect.currentframe().f_back.f_lineno


class AxisMr:
    def __init__(self, axisCom, url_string=None):
        self.axisCom = axisCom
        self.url_string = url_string
        self.hasFieldSPAM = None
        start_seconds = time.time()
        end_seconds = start_seconds + 30
        now = start_seconds
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} AxisMr.__init__ start url_string={url_string}"
        )
        while now < end_seconds:
            # Dummy read to give the IOC time to start
            self.hasFieldACCSwithVBASfix = False
            try:
                vers = float(self.axisCom.get(".VERS"))
                # 7.09 is rounded to 7.0900001xxx; use 7.091
                if (vers >= 6.94 and vers <= 7.091) or (vers >= 7.23 and vers < 7.29):
                    self.hasFieldACCS = True
                    self.hasFieldSPAM = True
                    self.hasFieldMFLG = True
                    self.hasROlimit = True
                    self.isMotorMasterAxis = False
                else:
                    self.hasFieldACCS = False
                    self.hasFieldACCSwithVBASfix = False
                    self.hasFieldSPAM = False
                    self.hasFieldMFLG = False
                    self.hasROlimit = False
                    self.isMotorMasterAxis = True
                # Upstream has ACCS/ACCL which takes VBAS into account
                # ess-master has this since 7.11
                if vers >= 7.23:
                    self.hasFieldACCSwithVBASfix = True
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} AxisMr.__init__ isMotorMasterAxis={self.isMotorMasterAxis} hasFieldACCS={self.hasFieldACCS} hasFieldACCSwithVBASfix={self.hasFieldACCSwithVBASfix}"
                )
                return
            except:  # noqa: E722
                pass
            time.sleep(polltime)
            now = time.time()
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} AxisMr.__init__ failed url_string={url_string}"
            )
        raise Exception("wait_for = 0 get None")

    MSTA_BIT_HOMED = 1 << (15 - 1)  # 4000
    MSTA_BIT_MINUS_LS = 1 << (14 - 1)  # 2000
    MSTA_BIT_COMM_ERR = 1 << (13 - 1)  # 1000
    MSTA_BIT_GAIN_SUPPORT = 1 << (12 - 1)  # 0800
    MSTA_BIT_MOVING = 1 << (11 - 1)  # 0400
    MSTA_BIT_PROBLEM = 1 << (10 - 1)  # 0200
    MSTA_BIT_PRESENT = 1 << (9 - 1)  # 0100
    MSTA_BIT_HOME = 1 << (8 - 1)  # 0080
    MSTA_BIT_SLIP_STALL = 1 << (7 - 1)  # 0040
    MSTA_BIT_AMPON = 1 << (6 - 1)  # 0020
    MSTA_BIT_UNUSED = 1 << (5 - 1)  # 0010
    MSTA_BIT_HOMELS = 1 << (4 - 1)  # 0008
    MSTA_BIT_PLUS_LS = 1 << (3 - 1)  # 0004
    MSTA_BIT_DONE = 1 << (2 - 1)  # 0002
    MSTA_BIT_DIRECTION = 1 << (1 - 1)  # 0001

    MIP_BIT_JOGF = 0x0001
    MIP_BIT_JOGR = 0x0002
    MIP_BIT_JOG_BL1 = 0x0004
    MIP_BIT_HOMF = 0x0008
    MIP_BIT_HOMR = 0x0010
    MIP_BIT_MOVE = 0x0020
    MIP_BIT_RETRY = 0x0040
    MIP_BIT_LOAD_P = 0x0080
    MIP_BIT_MOVE_BL = 0x0100
    MIP_BIT_STOP = 0x0200
    MIP_BIT_DELAY_REQ = 0x0400
    MIP_BIT_DELAY_ACK = 0x0800
    MIP_BIT_JOG_REQ = 0x1000
    MIP_BIT_JOG_STOP = 0x2000
    MIP_BIT_JOG_BL2 = 0x4000
    MIP_BIT_EXTERNAL = 0x8000

    MF_HOME_ON_LS = 1
    MF_LS_RAMPDOWN = 1 << 1
    MF_NO_STOP_ONLS = 1 << 2
    MF_DRIVER_USES_EGU = 1 << 3
    MF_ADJ_AFTER_HOMED = 1 << 4
    MF_NTM_UPDATE = 1 << 5
    MF_NOT_HOMED_PROBLEM = 1 << 6
    MF_NO_TWEAK_ONLS = 1 << 7

    # Values to be used for backlash test
    # Note: Make sure to use different values to hae a good
    # test coverage

    myMRES = 1.0
    myDIR = 0
    myOFF = 0.0
    myVELO = 10.0  # positioning velocity
    myACCL = 1.0  # Time to VELO, seconds
    myAR = myVELO / myACCL  # acceleration, mm/sec^2

    myJVEL = 5.0  # Jogging velocity
    myJAR = 6.0  # Jogging acceleration, mm/sec^2

    myBVEL = 2.0  # backlash velocity
    myBACC = 1.5  # backlash acceleration, seconds
    myBAR = myBVEL / myBACC  # backlash acceleration, mm/sec^2
    myRTRY = 3
    myDLY = 0.0
    myBDST = 24.0  # backlash destination, mm
    myFRAC = 1.0  #
    myPOSlow = 48  #
    myPOSmid = 72  # low + BDST
    myPOShig = 96  # low + 2*BDST

    def calcAlmostEqual(self, tc_no, expected, actual, maxdelta, doPrint=True):
        delta = math.fabs(expected - actual)
        delta <= maxdelta
        if delta <= maxdelta:
            inrange = True
        else:
            inrange = False
        if doPrint:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: calcAlmostEqual {tc_no} exp={expected} act={actual!r} delta={delta} maxdelta={maxdelta} inrange={inrange}"
            )
        return inrange

    def calcDVALfromVAL(self, tc_no, val):
        off = float(self.axisCom.get(".OFF"))
        dir = int(self.axisCom.get(".DIR"))
        if dir == 0:  # positive, the default
            dir = +1
        else:
            dir = -1  # negative
        dval = (val - off) * dir
        return dval

    def calcRVALfromVAL(self, tc_no, val):
        # We have one PV as a readback from the driver
        drvUseEGUmcu = int(self.axisCom.get("-DrvUseEGU-RB"))
        if self.hasFieldMFLG:
            mflg = int(self.axisCom.get(".MFLG"))
            if mflg & self.MF_DRIVER_USES_EGU:
                drvUseEGUmotorRecord = 1
            else:
                drvUseEGUmotorRecord = 0
            # What we have in the driver should match 'the motor flags' in the motorRecord
            if drvUseEGUmotorRecord != drvUseEGUmcu:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: calcRVALfromVAL mflg=0x{mflg:04x} drvUseEGUmotorRecord={drvUseEGUmotorRecord} drvUseEGUmcu={drvUseEGUmcu}"
                )
            assert drvUseEGUmotorRecord == drvUseEGUmcu
        mres = self.axisCom.get(".MRES")
        if drvUseEGUmcu == 1:
            # keep the sign
            if mres < 0.0:
                mres = -1.0
            else:
                mres = 1.0
        dval = self.calcDVALfromVAL(tc_no, val)
        rval = dval / mres
        return rval

    def calcHomeTimeOut(self, tc_no):
        axisCom = self.axisCom
        hlm = float(axisCom.get(".HLM"))
        llm = float(axisCom.get(".LLM"))
        if llm >= hlm:
            # This code is specific to ethercatmc.
            # However, this file is part of ethercatmc
            try:
                lll = float(axisCom.get("-CfgDLLM-RB"))
                hhh = float(axisCom.get("-CfgDHLM-RB"))
                llm = lll
                hlm = hhh
            except:  # noqa: E722
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} calcHomeTimeOut {tc_no} can not get soft limits"
                )

        range_postion = hlm - llm
        hvel = float(axisCom.get(".HVEL"))
        accl = axisCom.get(".ACCL")
        # Calculate the timeout, based on the driving range
        if range_postion > 0 and hvel > 0:
            timeout = 1 + 2 * range_postion / hvel + 2 * accl
        else:
            timeout = 180
        print(
            f"tc_no={tc_no} calcHomeTimeOut range_postion={range_postion} hvel={hvel} timeout={timeout:.2f}"
        )
        return timeout

    def calcTimeOut(self, destination, velocity):
        rbv = self.axisCom.get(".RBV", use_monitor=False)
        accl = self.axisCom.get(".ACCL", use_monitor=False)
        delta = math.fabs(destination - rbv)
        # timeout depends on the  accleration ramp
        timeout = 2 * accl + 2.0
        # if we have a velocity, use it
        if velocity != 0.0:
            timeout += delta / velocity
        else:
            timeout += 60.0
        print(
            f"calcTimeOut: rbv={rbv:.2f} destination={destination:.2f} velocity={velocity:.2f} timeout={timeout:.2f}"
        )
        return timeout

    def cmpUnlinkExpectedActualFile(self, tc_no, expFileName, actFileName):
        # compare actual and expFile
        sameContent = False
        wait_for_found = 3
        while wait_for_found > 0:
            try:
                file = open(expFileName)
                for line in file:
                    if line[-1] == "\n":
                        line = line[0:-1]
                    print(
                        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {expFileName}: {str(line)}"
                    )
                file.close()
                file = open(actFileName)
                for line in file:
                    if line[-1] == "\n":
                        line = line[0:-1]
                        if line == "EOF" and wait_for_found > 1:
                            wait_for_found = 1
                    print(
                        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {actFileName}: {str(line)}"
                    )
                file.close()
                sameContent = filecmp.cmp(expFileName, actFileName, shallow=False)
            except Exception as e:
                print(
                    "%s: cmpUnlinkExpectedActualFile expFileName=%s actFileName=%s wait_for_found=%f"
                    % (tc_no, expFileName, actFileName, wait_for_found)
                )
                print(str(e))
                time.sleep(0.5)

            if sameContent:
                os.unlink(expFileName)
                os.unlink(actFileName)
                return sameContent

            time.sleep(polltime)
            wait_for_found -= polltime

        return sameContent

    def doSTUPandSYNC(self, tc_no):
        self.waitForMipZero(tc_no, 2)
        self.waitForValueChanged(tc_no, ".STUP", 0, 0.0, 3.0)
        self.axisCom.put(".STUP", 1)
        self.waitForValueChanged(tc_no, ".STUP", 0, 0.0, 3.0)

        self.axisCom.put(".SYNC", 1)
        rbv = self.axisCom.get(".RBV", use_monitor=False)
        self.waitForValueChanged(tc_no, ".VAL", rbv, 0.1, 2.0)
        msta = int(self.axisCom.get(".MSTA", use_monitor=False))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} doSTUPandSYNC msta={self.getMSTAtext(msta)}"
        )

    def getFieldSPAM(
        self,
        tc_no,
    ):
        if self.hasFieldSPAM:
            return self.axisCom.get(".SPAM")
        return None

    def getIsMotorMaster(
        self,
        tc_no,
    ):
        return self.isMotorMasterAxis

    def getMIPtext(self, mip):
        ret = ""
        if mip & self.MIP_BIT_JOGF:
            ret = ret + "JOGF "
        if mip & self.MIP_BIT_JOGR:
            ret = ret + "JOGR "
        if mip & self.MIP_BIT_JOG_BL1:
            ret = ret + "JOG_BL1 "
        if mip & self.MIP_BIT_HOMF:
            ret = ret + "HOMF "
        if mip & self.MIP_BIT_HOMR:
            ret = ret + "HOMR "
        if mip & self.MIP_BIT_MOVE:
            ret = ret + "MOVE "
        if mip & self.MIP_BIT_LOAD_P:
            ret = ret + "LOAD_P "
        if mip & self.MIP_BIT_MOVE_BL:
            ret = ret + "MOVE_BL "
        if mip & self.MIP_BIT_STOP:
            ret = ret + "STOP "
        if mip & self.MIP_BIT_DELAY_REQ:
            ret = ret + "DELAY_REQ "
        if mip & self.MIP_BIT_DELAY_ACK:
            ret = ret + "DELAY_ACK "
        if mip & self.MIP_BIT_JOG_REQ:
            ret = ret + "JOG_REQ "
        if mip & self.MIP_BIT_JOG_STOP:
            ret = ret + "JOG_STOP "
        if mip & self.MIP_BIT_JOG_BL2:
            ret = ret + "JOG_BL2 "
        if mip & self.MIP_BIT_EXTERNAL:
            ret = ret + "EXTERNAL "
        return ret.rstrip()

    def getMSTAtext(self, msta):
        ret = ""
        if msta & self.MSTA_BIT_HOMED:
            ret = ret + "Hmd"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_MINUS_LS:
            ret = ret + "Lls"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_GAIN_SUPPORT:
            ret = ret + "Gai"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_MOVING:
            ret = ret + "Mov"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_PROBLEM:
            ret = ret + "Prb"
        else:
            ret = ret + "."
        if msta & self.MSTA_BIT_PRESENT:
            ret = ret + "Enc"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_HOME:
            ret = ret + "Hom"
        else:
            ret = ret + ".."
        if msta & self.MSTA_BIT_SLIP_STALL:
            ret = ret + "Slp"
        else:
            ret = ret + "...."
        if msta & self.MSTA_BIT_AMPON:
            ret = ret + "Amp"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_HOMELS:
            ret = ret + "Hsw"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_PLUS_LS:
            ret = ret + "Hls"
        else:
            ret = ret + "..."
        if msta & self.MSTA_BIT_DONE:
            ret = ret + "Don"
        else:
            ret = ret + "..."
        return ret.rstrip()

    def initializeMotorRecordOneField(self, tc_no, field_name, value):
        oldVal = self.axisCom.get(field_name)

        if oldVal is not None:
            print(
                "%s: initializeMotorRecordOneField field=%s oldVal=%f value=%f"
                % (tc_no, field_name, oldVal, value)
            )
            if oldVal != value:
                self.axisCom.put(field_name, value)
        else:
            print(
                "%s: initializeMotorRecordOneField field=%s not found value=%f"
                % (tc_no, field_name, value)
            )

    def initializeMotorRecordSimulatorAxis(self, tc_no):
        self.initializeMotorRecordOneField(tc_no, ".VMAX", 50.0)
        self.initializeMotorRecordOneField(tc_no, ".VELO", 20.0)
        self.initializeMotorRecordOneField(tc_no, ".ACCL", 5.0)
        self.initializeMotorRecordOneField(tc_no, ".JVEL", 5.0)
        self.initializeMotorRecordOneField(tc_no, ".JAR", 20.0)

        self.initializeMotorRecordOneField(tc_no, ".RDBD", 0.1)
        # self.initializeMotorRecordOneField( tc_no, '.SPDB', 0.1)
        self.initializeMotorRecordOneField(tc_no, ".BDST", 0.0)

        cfgDHLM = 53.0
        cfgDLLM = -54.0
        self.setSoftLimitsOff(tc_no)
        self.setValueOnSimulator(tc_no, "fSimForcePos", 0.0)
        self.setValueOnSimulator(tc_no, "fHighSoftLimitPos", cfgDHLM)
        self.setValueOnSimulator(tc_no, "fLowSoftLimitPos", cfgDLLM)
        self.initializeMotorRecordOneField(tc_no, "-CfgDHLM-En", 1)
        self.initializeMotorRecordOneField(tc_no, "-CfgDLLM-En", 1)
        self.initializeMotorRecordOneField(tc_no, ".DHLM", cfgDHLM)
        self.initializeMotorRecordOneField(tc_no, ".DLLM", cfgDLLM)

    def jogCalcTimeout(self, tc_no, direction):
        jvel = self.axisCom.get(".JVEL")
        hlm = self.axisCom.get(".HLM")
        llm = self.axisCom.get(".LLM")
        rbv = self.axisCom.get(".RBV")
        accl = self.axisCom.get(".ACCL")
        deltah = math.fabs(hlm - rbv)
        deltal = math.fabs(llm - rbv)
        # TODO: we could use at the DIR field, which delta to use
        # This can be done in a cleanup
        if direction > 0:
            delta = deltah
        else:
            delta = deltal
        # TODO: add JAR to the calculation
        time_to_wait = delta / jvel + 2 * accl + 2.0
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: jogCalcTimeout jogDirection={direction} rbv={rbv:.2f} delta={delta:.2f} jvel={jvel:.2f} time_to_wait={time_to_wait:.2f}"
        )
        return time_to_wait

    def jogDirection(self, tc_no, direction):
        time_to_wait = self.jogCalcTimeout(tc_no, direction)
        return self.jogDirectionTimeout(tc_no, direction, time_to_wait)

    #    def movePosition(self, tc_no, destination, velocity, acceleration):
    #        time_to_wait = 30
    #        if velocity > 0:
    #            distance = math.fabs(self.axisCom.get(".RBV") - destination)
    #            time_to_wait += distance / velocity + 2 * acceleration
    #        self.axisCom.put(".VAL", destination)
    #        done = self.waitForStartAndDone(str(tc_no) + " movePosition", time_to_wait)

    def jogDirectionTimeout(self, tc_no, direction, time_to_wait):
        start_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
        start_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: jogDirectionTimeout direction={direction} time_to_wait={time_to_wait:.2f} start_change_cnt_dmov_true={start_change_cnt_dmov_true} start_change_cnt_dmov_false={start_change_cnt_dmov_false}"
        )
        if direction > 0:
            self.axisCom.put(".JOGF", 1)
        else:
            self.axisCom.put(".JOGR", 1)
        self.waitForStartAndDone(str(tc_no) + " jogDirection", 30 + time_to_wait + 3.0)
        mov1_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
        mov1_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
        num_change_cnt_dmov_true = (
            mov1_change_cnt_dmov_true - start_change_cnt_dmov_true
        )
        num_change_cnt_dmov_false = (
            mov1_change_cnt_dmov_false - start_change_cnt_dmov_false
        )

        jogf = int(self.axisCom.get(".JOGF"))
        jogr = int(self.axisCom.get(".JOGR"))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: jogr={jogr} jogf={jogf } num_change_cnt_dmov_true={num_change_cnt_dmov_true} num_change_cnt_dmov_false={num_change_cnt_dmov_false}"
        )
        return (
            jogf == 0
            and jogr == 0
            and num_change_cnt_dmov_true == 1
            and num_change_cnt_dmov_false == 1
        )

    def motorInitAllForBDST(self, tc_no):
        startPos = 0.0
        # The next is needed to change MRES_23/24 further down
        self.setValueOnSimulator(tc_no, "nAmplifierPercent", 0)
        self.setValueOnSimulator(tc_no, "bAxisHomed", 1)
        self.setValueOnSimulator(tc_no, "fLowHardLimitPos", -120)
        self.setValueOnSimulator(tc_no, "fHighHardLimitPos", 120)
        self.setValueOnSimulator(tc_no, "setMRES_23", 0)
        self.setValueOnSimulator(tc_no, "setMRES_24", 0)
        self.setValueOnSimulator(tc_no, "fSimForcePos", startPos)
        self.setValueOnSimulator(tc_no, "nAmplifierPercent", 100)

        self.axisCom.put("-ErrRst", 1)
        # Prepare parameters for jogging and backlash
        self.setSoftLimitsOff(tc_no)
        self.axisCom.put(".MRES", self.myMRES)
        self.axisCom.put(".DIR", self.myDIR)
        self.axisCom.put(".OFF", self.myOFF)
        self.axisCom.put(".VELO", self.myVELO)
        self.axisCom.put(".ACCL", self.myACCL)

        self.axisCom.put(".JVEL", self.myJVEL)
        self.axisCom.put(".JAR", self.myJAR)

        self.axisCom.put(".BVEL", self.myBVEL)
        self.axisCom.put(".BACC", self.myBACC)
        self.axisCom.put(".BDST", self.myBDST)
        self.axisCom.put(".FRAC", self.myFRAC)
        self.axisCom.put(".RTRY", self.myRTRY)
        self.axisCom.put(".RMOD", motorRMOD_D)
        self.axisCom.put(".DLY", self.myDLY)
        self.axisCom.put(".SYNC", 1)
        self.waitForValueChanged(tc_no, ".VAL", startPos, 0.1, 2.0)

    def motorInitAllForBDSTIfNeeded(self, tc_no):
        init_needed = 0
        if self.axisCom.get(".MRES") != self.myMRES:
            init_needed = init_needed + 1
        if self.axisCom.get(".DIR") != self.myDIR:
            init_needed = init_needed + 2
        if self.axisCom.get(".OFF") != self.myOFF:
            init_needed = init_needed + 1
        if self.axisCom.get(".VELO") != self.myVELO:
            init_needed = init_needed + 4
        if self.axisCom.get(".ACCL") != self.myACCL:
            init_needed = init_needed + 8
        if self.axisCom.get(".JVEL") != self.myJVEL:
            init_needed = init_needed + 16
        if self.axisCom.get(".JAR") != self.myJAR:
            init_needed = init_needed + 32
        if self.axisCom.get(".BVEL") != self.myBVEL:
            init_needed = init_needed + 64
        if self.axisCom.get(".BACC") != self.myBACC:
            init_needed = init_needed + 128
        if self.axisCom.get(".RTRY") != self.myRTRY:
            self.axisCom.put(".RTRY", self.myRTRY)
        if self.axisCom.get(".BDST") != self.myBDST:
            self.axisCom.put(".BDST", self.myBDST)
        if self.axisCom.get(".DLY") != self.myDLY:
            init_needed = init_needed + 256
        if self.axisCom.get(".HLM") != 0.0:
            init_needed = init_needed + 512
        if self.axisCom.get(".LLM") != 0.0:
            init_needed = init_needed + 1024
        if init_needed == 0:
            return
        debug_text = f"{tc_no}#{lineno()} init_needed=0x{init_needed:X}"
        self.axisCom.putDbgStrToLOG(debug_text, wait=True)
        self.motorInitAllForBDST(tc_no)

    def moveIntoLimitSwitchFromTestCase(
        self,
        tc_no,
        direction=0,
        movingMethod="",
        doDisableSoftLimit=True,
        setInfiniteSoftLimit=False,
        setDLYfield=None,
    ):
        self.axisCom.putDbgStrToLOG("Start " + str(int(tc_no)), wait=True)
        self.powerOnHomeAxis(tc_no)
        self.setSoftLimitsOn(tc_no, initAbsMinMax=True)
        if setDLYfield is not None:
            saved_DLY = self.axisCom.get(".DLY")
            self.axisCom.put(".DLY", setDLYfield)
        passed = self.moveIntoLS(
            tc_no=tc_no,
            direction=direction,
            movingMethod=movingMethod,
            doDisableSoftLimit=doDisableSoftLimit,
            setInfiniteSoftLimit=setInfiniteSoftLimit,
        )
        if setDLYfield is not None:
            self.axisCom.put(".DLY", saved_DLY)

        passed_str = "Passed " if passed is True else "Failed "
        self.axisCom.putDbgStrToLOG(passed_str + str(tc_no), wait=True)
        return passed

    # move into limit switch
    # We need different combinations (WIP)
    # - The direction (hit HLS or LLS)
    # - "switch soft limits off" -or- set a very high/low soft limit instead
    #    (or do nothing)
    # - Use motorRecords JOG or use model 3 moveVel/moveAbs
    # - Change the parameter from above before or after the movement started
    #
    def moveIntoLS(
        self,
        tc_no=0,
        direction=0,
        doDisableSoftLimit=True,
        setInfiniteSoftLimit=False,
        movingMethod="JOG",
    ):
        if tc_no < 0:
            raise ValueError(f"Expected tc_no to be greater than 0, but got {tc_no}")
        if direction not in {-1, 1}:
            raise ValueError(f"Expected 1 or -1, but got {direction}")

        old_VELO = self.axisCom.get(".VELO")
        vmax = self.axisCom.get(".VMAX")
        if vmax == 0.0:
            vmax = old_VELO
        jvel = self.axisCom.get(".JVEL")
        if jvel == 0.0:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} jvel={jvel}"
            )
            return False

        old_DHLM = self.axisCom.get("-CfgDHLM")
        old_DLLM = self.axisCom.get("-CfgDLLM")
        margin = 1.1
        if movingMethod == "MoveVel":
            movingFieldName = "-MoveVel"
        if direction == 1:
            softlimitFieldName = "-CfgDHLM"
            nearly_infinite = 999999.0
            soft_limit_pos = old_DHLM
            jog_start_pos = soft_limit_pos - jvel - margin
            lsActivetedFieldName = ".HLS"
            lsNotActiveFieldName = ".LLS"
            if movingMethod == "JOG":
                movingFieldName = ".JOGF"
                movingFieldValue = 1
            elif movingMethod == "MoveVel":
                movingFieldValue = jvel
            elif movingMethod == "DVAL":
                movingFieldName = ".DVAL"
                movingFieldValue = nearly_infinite
        elif direction == -1:
            softlimitFieldName = "-CfgDLLM"
            nearly_infinite = -999999.0
            soft_limit_pos = old_DLLM
            jog_start_pos = soft_limit_pos + jvel + margin
            lsActivetedFieldName = ".LLS"
            lsNotActiveFieldName = ".HLS"
            if movingMethod == "JOG":
                movingFieldName = ".JOGR"
                movingFieldValue = 1
            elif movingMethod == "MoveVel":
                movingFieldValue = 0 - jvel
            elif movingMethod == "MoveAbs":
                movingFieldValue = 0 - jvel
            elif movingMethod == "DVAL":
                movingFieldName = ".DVAL"
                movingFieldValue = nearly_infinite
        else:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} illegal direction={direction}"
            )
            return False

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} margin={margin}"
        )
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} direction={direction } jog_start_pos={jog_start_pos:f}"
        )
        dmov = int(self.axisCom.get(".DMOV", use_monitor=False))
        if dmov != 1:
            self.axisCom.put(".STOP", 1)
        # Go away from limit switch
        self.moveWait(tc_no, jog_start_pos)
        if movingMethod == "MoveAbs":
            movingFieldName = ".DVAL"
            self.axisCom.put(".VELO", jvel)
            movingFieldValue = nearly_infinite

        if doDisableSoftLimit:
            self.setSoftLimitsOff(tc_no, direction=direction)
            time_to_wait = self.jogCalcTimeout(tc_no, direction)
            self.axisCom.put(movingFieldName, movingFieldValue)
            self.waitForStartAndDone(str(tc_no), 30 + time_to_wait + 3.0)
        else:
            if setInfiniteSoftLimit:
                oldSoftLimitValue = self.axisCom.get(softlimitFieldName)
                self.axisCom.put(softlimitFieldName, nearly_infinite)

            time_to_wait = self.jogCalcTimeout(tc_no, direction)
            self.axisCom.put(movingFieldName, movingFieldValue)
            self.waitForStartAndDone(str(tc_no), 30 + time_to_wait + 3.0)

        # Get values, check them later
        lvio = int(self.axisCom.get(".LVIO"))
        mstaE = int(self.axisCom.get(".MSTA"))
        lsActivetedVal = int(self.axisCom.get(lsActivetedFieldName))
        lsNotActiveVal = int(self.axisCom.get(lsNotActiveFieldName))
        # Go away from limit switch
        self.moveWait(tc_no, jog_start_pos)
        self.axisCom.put(".VELO", old_VELO)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} msta={mstaE:x} msta={self.getMSTAtext(mstaE)} lvio={int(lvio)}"
        )

        if doDisableSoftLimit:
            self.setSoftLimitsOn(tc_no, old_DLLM, old_DHLM)
        if setInfiniteSoftLimit:
            self.axisCom.put(softlimitFieldName, oldSoftLimitValue)

        passed = True
        if (mstaE & self.MSTA_BIT_PROBLEM) != 0:
            errId = int(self.axisCom.get("-ErrId", use_monitor=False))
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} msta={mstaE:x} errId={errId:x}"
            )
            if errId == 0x4223:
                self.resetAxis(tc_no)
            else:
                passed = False

        if (lsNotActiveVal) != 0:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} wrong LS activated"
            )
            passed = False

        if (lsActivetedVal) == 0:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} LS was not activated"
            )
            passed = False

        return passed

    def moveWait(self, tc_no, destination, throw=True):
        start_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
        start_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
        timeout = 30
        rbv = self.axisCom.get(".RBV")
        acceleration = self.axisCom.get(".ACCL")
        velocity = self.axisCom.get(".VELO")
        timeout += 2 * acceleration + 1.0
        if velocity > 0:
            distance = math.fabs(rbv - destination)
            timeout += distance / velocity

        mov1_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
        mov1_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}:moveWait destination={destination:.2f} rbv={rbv:.2f} mov1_change_cnt_dmov_true={mov1_change_cnt_dmov_true} mov1_change_cnt_dmov_false={mov1_change_cnt_dmov_false}"
        )
        # The motorRecord will handle even situations like rbv=0.501 and val=5.00
        ret0 = self.axisCom.put(
            ".VAL", destination, wait=True, timeout=timeout, throw=throw
        )
        timeToWait = 1.0  # wait max 1 second for the callbacks
        while timeToWait > 0.0:
            mov1_change_cnt_dmov_true = self.axisCom.get_change_cnts("dmov_true")
            mov1_change_cnt_dmov_false = self.axisCom.get_change_cnts("dmov_false")
            num_change_cnt_dmov_true = (
                mov1_change_cnt_dmov_true - start_change_cnt_dmov_true
            )
            num_change_cnt_dmov_false = (
                mov1_change_cnt_dmov_false - start_change_cnt_dmov_false
            )
            if num_change_cnt_dmov_true == 1 and num_change_cnt_dmov_false == 1:
                timeToWait = 0.0
            else:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}:moveWait num_change_cnt_dmov_true={num_change_cnt_dmov_true} num_change_cnt_dmov_false={num_change_cnt_dmov_false} timeToWait={timeToWait:.2f}"
                )
                time.sleep(polltime)
                timeToWait -= polltime

        ret = ret0 and num_change_cnt_dmov_true == 1 and num_change_cnt_dmov_false == 1
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}:moveWait num_change_cnt_dmov_true={num_change_cnt_dmov_true} num_change_cnt_dmov_false={num_change_cnt_dmov_false} ret0={ret0} ret={ret}"
        )
        return ret

    def postMoveCheck(self, tc_no):
        # Check the motor for the correct state at the end of move.
        # Note: lvio is, when the EPICS IOC starts and the motor
        # is where it should be kept as 1
        # This is probably a bug. Ignore lvio for now
        val = float(self.axisCom.get(".VAL"))
        rbv = float(self.axisCom.get(".RBV", use_monitor=False))
        dmov = int(self.axisCom.get(".DMOV"))
        movn = int(self.axisCom.get(".MOVN"))
        stat = int(self.axisCom.get(".STAT"))
        sevr = int(self.axisCom.get(".SEVR"))
        miss = int(self.axisCom.get(".MISS"))
        rhls = int(self.axisCom.get(".RHLS"))
        rlls = int(self.axisCom.get(".RLLS"))

        ret = (
            dmov == 1
            and movn == 0
            and stat == 0
            and sevr == 0
            and miss == 0
            and rhls == 0
            and rlls == 0
        )

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} postMoveCheck dmov={dmov:d} movn={movn:d} stat={stat:X} sevr={sevr} miss={miss:d} rhls={rhls:d} rlls={rlls:d} val={val:.2f} rbv={rbv:.2f} ret={ret}"
        )
        return ret

    def powerOnHomeAxis(self, tc_no):
        self.setCNENandWait(tc_no, 1)
        msta = int(self.axisCom.get(".MSTA"))
        if not (msta & self.MSTA_BIT_HOMED):
            time_to_wait = self.calcHomeTimeOut(tc_no)
            msta = int(self.axisCom.get(".MSTA"))
            # If we are sitting on the High limit switch, use HOMR
            if msta & self.MSTA_BIT_PLUS_LS:
                self.axisCom.put(".HOMR", 1)
            else:
                self.axisCom.put(".HOMF", 1)
                self.waitForStartAndDone(tc_no, time_to_wait)
            msta = int(self.axisCom.get(".MSTA"))
            assert msta & self.MSTA_BIT_HOMED

    def resetAxis(self, tc_no):
        wait_for_ErrRst = 5
        err = int(self.axisCom.get("-Err", use_monitor=False))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} resetAxis err={int(err)}"
        )

        self.axisCom.put("-ErrRst", 1)
        while wait_for_ErrRst > 0:
            wait_for_ErrRst -= polltime
            err = int(self.axisCom.get("-Err", use_monitor=False))
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} wait_for_ErrRst={wait_for_ErrRst:.2f} err=0X{err:X}"
            )
            if not err:
                return True
            time.sleep(polltime)
            wait_for_ErrRst -= polltime
        return False

    def setCNENandWait(self, tc_no, cnen):
        wait_for_power_changed = 6.0
        # capv_self.capvput(
        #     + "-DbgStrToLOG", "CNEN=" + str(cnen) + " " + tc_no[0:20], wait=True
        # )
        self.axisCom.put(".CNEN", cnen)
        while wait_for_power_changed > 0:
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_power_changed={wait_for_power_changed:.2f} cnen={cnen} msta={msta:04x} {self.getMSTAtext(msta)}"
            print(debug_text)
            if cnen and (msta & self.MSTA_BIT_AMPON):
                return
            if not cnen and not (msta & self.MSTA_BIT_AMPON):
                return
            time.sleep(polltime)
            wait_for_power_changed -= polltime
        raise Exception(debug_text)

    def setFieldSPAM(self, tc_no, value):
        if self.hasFieldSPAM:
            self.axisCom.put(".SPAM", value)

    def setMotorStartPos(self, tc_no, startpos):
        self.setValueOnSimulator(tc_no, "fSimForcePos", startpos)
        self.doSTUPandSYNC(tc_no)
        maxDelta = 0.1
        timeout = 3.0
        valueVALok = self.waitForValueChanged(
            tc_no, ".VAL", startpos, maxDelta, timeout
        )
        return valueVALok

    def setSoftLimitsOff(self, tc_no, direction=-1):
        actDHLM = float(self.axisCom.get(".DHLM", use_monitor=False))
        actDLLM = float(self.axisCom.get(".DLLM", use_monitor=False))

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: setSoftLimitsOff direction={direction} actDHLM={actDHLM} actDLLM={actDLLM}"
        )
        # switch off the controller soft limits
        if direction == 0:
            self.axisCom.put("-CfgDLLM-En", 0, wait=True)
            self.axisCom.put("-CfgDHLM-En", 0, wait=True)
        else:
            self.axisCom.put("-CfgDHLM-En", 0, wait=True)
            self.axisCom.put("-CfgDLLM-En", 0, wait=True)

        maxTime = 10  # seconds maximum to let read only parameters ripple through
        maxDelta = 0.05  # 5 % error tolerance margin
        while maxTime > 0:
            self.axisCom.put(".DLLM", 0.0)
            self.axisCom.put(".DHLM", 0.0)

            actDHLM = float(self.axisCom.get(".DHLM", use_monitor=False))
            actDLLM = float(self.axisCom.get(".DLLM", use_monitor=False))

            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: setSoftLimitsOff dhlm={actDHLM} dllm={actDLLM}"
            )
            resH = self.calcAlmostEqual(tc_no, 0.0, actDHLM, maxDelta)
            resL = self.calcAlmostEqual(tc_no, 0.0, actDLLM, maxDelta)
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: setSoftLimitsOff actDHLM={actDHLM} actDLLM={actDLLM} resH={resH} resL={resL}"
            print(debug_text)
            if resH and resL:
                return

            time.sleep(polltime)
            maxTime = maxTime - polltime
        raise Exception(debug_text)
        assert False

    def setSoftLimitsOn(
        self, tc_no, low_limit=0.0, high_limit=0.0, initAbsMinMax=False
    ):
        if initAbsMinMax:
            high_limit = self.axisCom.get("-CfgPMAX-RB")
            low_limit = self.axisCom.get("-CfgPMIN-RB")

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}: setSoftLimitsOn low_limit={low_limit} high_limit={high_limit} initAbsMinMax={initAbsMinMax}"
        )
        # switch on the controller soft limits
        try:
            self.axisCom.put("-CfgDHLM", high_limit, wait=True, timeout=2)
            self.axisCom.put("-CfgDLLM", low_limit, wait=True, timeout=2)
            self.axisCom.put("-CfgDHLM-En", 1, wait=True, timeout=2)
            self.axisCom.put("-CfgDLLM-En", 1, wait=True, timeout=2)
        finally:
            oldRBV = self.axisCom.get(".RBV")

        if oldRBV < 0:
            self.axisCom.put(".LLM", low_limit)
            self.axisCom.put(".HLM", high_limit)
        else:
            self.axisCom.put(".HLM", high_limit)
            self.axisCom.put(".LLM", low_limit)

    def setValueOnSimulator(self, tc_no, var, value):
        var = str(var)
        value = str(value)
        outStr = "Sim.this." + var + "=" + value
        print(
            "%s/%s: DbgStrToMCU var=%s value=%s outStr=%s"
            % (tc_no, self.url_string, var, value, outStr)
        )
        if not "TODOXXXX".startswith("pva://"):
            lenOutStr = len(outStr)
            if lenOutStr >= 40:
                print(
                    "%s: setValueOnSimulator lenOutStr=%d outStr=%s"
                    % (tc_no, lenOutStr, outStr)
                )
                assert len(outStr) < 40
        self.axisCom.put("-DbgStrToMCU", outStr, wait=True)
        stat = int(self.axisCom.get("-DbgStrToMCU.STAT", use_monitor=False))
        sevr = int(self.axisCom.get("-DbgStrToMCU.SEVR", use_monitor=False))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: setValueOnSimulator var={var} value={value} stat={stat} sevr={sevr}"
        )
        if stat != 0 or sevr != 0:
            debug_text = f"stat={stat} sevr={sevr}"
            raise Exception(debug_text)

    def verifyRBVinsideRDBD(self, tc_no, position):
        """"""
        rdbd = self.axisCom.get(".RDBD")
        rbv = self.axisCom.get(".RBV", use_monitor=False)

        if (rbv < position - rdbd) or (rbv > position + rdbd):
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: verifyRBVinsideRDBD position={position} rbv={rbv:.2f} rdbd={rdbd}"
            )
            return False
        return True

    def waitForMipZero(self, tc_no, wait_for_mip_zero):
        while wait_for_mip_zero > -10.0:  # Extra long wait
            wait_for_mip_zero -= polltime
            mip = int(self.axisCom.get(".MIP", use_monitor=False))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_mip_zero={wait_for_mip_zero:.2f} mip={self.getMIPtext(mip)} (0x{mip:04x}) rbv={rbv:.2f}"
            print(debug_text)
            if not mip:
                return
            time.sleep(polltime)
            wait_for_mip_zero -= polltime
        raise Exception(debug_text)

    def waitForPowerOff(self, tc_no, wait_for_powerOff):
        while wait_for_powerOff > 0:
            wait_for_powerOff -= polltime
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            powerOn = msta & self.MSTA_BIT_AMPON
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_powerOff={wait_for_powerOff:.2f} powerOn={int(powerOn)}"
            print(debug_text)
            if not powerOn:
                return True
            time.sleep(polltime)
            wait_for_powerOff -= polltime
        raise Exception(debug_text)

    def waitForPowerOn(self, tc_no, wait_for_powerOn):
        while wait_for_powerOn > 0:
            wait_for_powerOn -= polltime
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            powerOn = msta & self.MSTA_BIT_AMPON
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_powerOn={wait_for_powerOn:.2f} powerOn={int(powerOn)}"
            print(debug_text)
            if powerOn:
                return
            time.sleep(polltime)
            wait_for_powerOn -= polltime
        raise Exception(debug_text)

    def waitForStart(self, tc_no, wait_for_start, throw=True):
        while wait_for_start > 0:
            wait_for_start -= polltime
            dmov = int(self.axisCom.get(".DMOV", use_monitor=False))
            movn = int(self.axisCom.get(".MOVN", use_monitor=False))
            rbv = self.axisCom.get(".RBV")
            val = self.axisCom.get(".VAL")
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_start={wait_for_start:.2f} dmov={dmov} movn={movn} val={val:.2f} rbv={rbv:.2f}"
            print(debug_text)
            if movn and not dmov:
                return True
            time.sleep(polltime)
            wait_for_start -= polltime
        if throw:
            raise Exception(debug_text)
        else:
            return False

    def waitForStartAndDone(self, tc_no, wait_for_done, throw=True):
        val = self.axisCom.get(".VAL", use_monitor=False)
        wait_for_start = 2
        while wait_for_start > 0:
            wait_for_start -= polltime
            dmov = int(self.axisCom.get(".DMOV"))
            movn = int(self.axisCom.get(".MOVN"))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_start_and_done_start={wait_for_start:.2f} dmov={dmov} movn={movn} val={val:.2f} rbv={rbv:.2f}"
            print(debug_text)
            if movn or not dmov:
                wait_for_start = 0
            else:
                time.sleep(polltime)

        wait_for_done = math.fabs(wait_for_done)  # negative becomes positive
        wait_for_done += 5  # One extra second for rounding
        while wait_for_done > 0:
            dmov = int(self.axisCom.get(".DMOV"))
            movn = int(self.axisCom.get(".MOVN"))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            mipTxt = self.getMIPtext(int(self.axisCom.get(".MIP", use_monitor=False)))
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_start_and_done_done={wait_for_done:.2f} dmov={dmov} movn={movn} rbv={rbv:.2f} mipTxt={mipTxt}"
            print(debug_text)
            if dmov and not movn:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_start_and_done_done: return OK"
                )
                return True
            time.sleep(polltime)
            wait_for_done = wait_for_done - polltime
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_start_and_done_done: raise Exception"
        )
        if throw:
            raise Exception(debug_text)
        return False

    def waitForStop(self, tc_no, wait_for_stop, throw=True):
        while wait_for_stop > 0:
            dmov = int(self.axisCom.get(".DMOV", use_monitor=False))
            movn = int(self.axisCom.get(".MOVN", use_monitor=False))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: wait_for_stop={wait_for_stop:.2f} dmov={dmov} movn={movn} rbv={rbv:.2f}"
            print(debug_text)
            if not movn and dmov:
                return True
            time.sleep(polltime)
            wait_for_stop -= polltime
        if throw:
            raise Exception(debug_text)
        else:
            return False

    def waitForValueChanged(
        self, tc_no, field_name, expVal, maxDelta, time_to_wait, debugPrint=True
    ):
        while time_to_wait > 0:
            actVal = self.axisCom.get(field_name, use_monitor=False)
            inrange = self.calcAlmostEqual(
                tc_no, expVal, actVal, maxDelta, doPrint=False
            )
            time_to_wait -= polltime
            if debugPrint or time_to_wait <= 0:
                debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: waitForValueChanged time_to_wait={time_to_wait:.2f} field_name={field_name} expVal={expVal:.2f} actVal={actVal:.2f} maxDelta={maxDelta:.3f} inrange={inrange}"
                print(debug_text)
                debugPrint = False
            if inrange:
                return True
            time.sleep(polltime)
        return False

    def waitForValueChangedInt32(
        self, tc_no, field_name, expVal, time_to_wait, debugPrint=True
    ):
        inrange = False
        while time_to_wait > 0:
            actVal = int(self.axisCom.get(field_name, use_monitor=False))
            if ((int(expVal) & 0xFFFFFFFF) ^ (int(actVal) & 0xFFFFFFFF)) == 0:
                inrange = True
            if debugPrint:
                debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: waitForValueChanged time_to_wait={time_to_wait:.2f} field_name={field_name} expVal={expVal:X} actVal={actVal:X} inrange={inrange}"
                print(debug_text)
            if inrange:
                return True
            time.sleep(polltime)
            time_to_wait -= polltime
        return False

    def writeExpFileJOG_BDST(
        self,
        tc_no,
        expFileName,
        myDirection,
        frac,
        encRel,
        maxcnt,
        motorStartPos,
        motorEndPos,
    ):
        # vers = float(self.axisCom.get(".VERS"))
        # motor_master = False
        # if vers > 7.19:
        #    motor_master = True

        bdst = float(self.axisCom.get(".BDST", timeout=2.0, use_monitor=False))
        debug_text = f"{tc_no}#{lineno()} Start={motorStartPos} End={motorEndPos} bdst={bdst} encRel={encRel}"
        self.axisCom.putDbgStrToLOG(debug_text, wait=True)
        # Create a "expected" file
        expFile = open(expFileName, "w")

        # The jogging command
        line1 = (
            "move velocity direction=%d max_velocity=%g acceleration=%g motorPosNow=%g"
            % (myDirection, self.myJVEL, self.myJAR, motorStartPos)
        )
        deltaForth = bdst
        # The record tells us to go "delta * frac". Once we have travelled, we are too far
        # The record will read that we are too far, and ask to go back "too far", overcompensated
        # again with frac
        deltaBack = deltaForth * frac
        if encRel:
            # Move back in relative mode
            line2 = (
                "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                % (0 - deltaForth, self.myVELO, self.myAR, motorEndPos)
            )
            # Move relative forward with backlash parameters
            motorPosNow = motorEndPos - deltaForth
            line3 = (
                "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                % (deltaBack, self.myBVEL, self.myBAR, motorPosNow)
            )
            expFile.write(f"{line1}\n{line2}\n{line3}\n")
            rtry = self.axisCom.get(".RTRY", use_monitor=False)
            rcnt = self.axisCom.get(".RCNT", use_monitor=False)
            while rtry > rcnt and frac != 1.0:
                # motorRecord will do retries.
                motorPosNow = motorPosNow + deltaBack
                # Backlash is DVAL - BDST (with DVAL == motorEndPos)
                newMotorPos = motorEndPos - bdst
                commandedDelta = (newMotorPos - motorPosNow) * frac
                line4 = (
                    "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                    % (commandedDelta, self.myVELO, self.myAR, motorPosNow)
                )
                expFile.write(f"{line4}\n")
                motorPosNow = motorPosNow + commandedDelta
                # Move the opposite direction; motorRecord will multiply with frac (again)
                newMotorPos = motorEndPos
                commandedDelta = (newMotorPos - motorPosNow) * frac
                line5 = (
                    "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                    % (commandedDelta, self.myBVEL, self.myBAR, motorPosNow)
                )
                expFile.write(f"{line5}\n")
                rtry = rtry - 1

        else:
            expFile.write(f"{line1}\n")
            startPosLine2 = motorEndPos
            # Move back in positioning mode
            while maxcnt > 0:
                line2 = (
                    "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                    % (motorEndPos - bdst, self.myVELO, self.myAR, startPosLine2)
                )
                # Move forward with backlash parameters times frac
                # double currpos = pmr->dval / pmr->mres;
                # double newpos = bpos + pmr->frac * (currpos - bpos);
                posNow = motorEndPos - bdst
                deltaToMove = bdst * frac
                endPosLine3 = posNow + deltaToMove
                line3 = (
                    "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                    % (endPosLine3, self.myBVEL, self.myBAR, posNow)
                )
                startPosLine2 = endPosLine3
                expFile.write(f"{line2}\n{line3}\n")
                maxcnt = maxcnt - 1

        expFile.write("EOF\n")
        expFile.close()

    def writeExpFileRMOD_X(
        self,
        tc_no,
        rmod,
        expFile,
        frac,
        encRel,
        motorStartPos,
        motorEndPos,
        need_007_017_tweak=False,
    ):
        if motorEndPos - motorStartPos > 0:
            directionOfMove = 1
        else:
            directionOfMove = -1
        bdst = self.axisCom.get(".BDST", timeout=2.0, use_monitor=False)
        if bdst > 0.0:
            directionOfBL = 1
        elif bdst < 0.0:
            directionOfBL = -1
        else:
            directionOfBL = 0

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}: writeExpFileRMOD_X encRel={encRel} motorStartPos={motorStartPos} motorEndPos={motorEndPos} directionOfMove{directionOfMove} directionOfBL={directionOfBL}"
        )
        if rmod == motorRMOD_I:
            maxcnt = 1  # motorRMOD_I means effecttivly "no retry"
            encRel = 0
        else:
            maxcnt = 1 + int(self.axisCom.get(".RTRY"))

        cnt = 0
        # motorRecord uses a margin: one step
        rbdst1 = abs(self.myBDST) + 1.0 / abs(self.myMRES)
        debug_text = f"{tc_no}#{lineno()} Start={motorStartPos} End={motorEndPos} rbdst1={rbdst1}"
        self.axisCom.putDbgStrToLOG(debug_text, wait=True)
        debug_text = f"{tc_no}#{lineno()} dirMove={directionOfMove} dirBL={directionOfBL} BDST={self.myBDST}"
        self.axisCom.putDbgStrToLOG(debug_text, wait=True)
        while cnt < maxcnt:
            line1 = ""
            line2 = ""
            factor = 1.0
            if cnt > 1:
                if encRel:
                    if rmod == motorRMOD_A:
                        # From motorRecord.cc:
                        # factor = (pmr->rtry - pmr->rcnt + 1.0) / pmr->rtry;
                        factor = 1.0 * (self.myRTRY - cnt + 1.0) / self.myRTRY
                    elif rmod == motorRMOD_G:
                        # factor = 1 / pow(2.0, (pmr->rcnt - 1));
                        rcnt_1 = cnt - 1
                        factor = 1.0
                        while rcnt_1 > 0:
                            factor = factor / 2.0
                            rcnt_1 -= 1
            if (
                abs(motorEndPos - motorStartPos) < rbdst1
                and directionOfMove == directionOfBL
            ):
                moveWithBL = True
            else:
                moveWithBL = False
            debug_text = f"{tc_no}#{lineno()} moveWithBL={moveWithBL}"
            self.axisCom.putDbgStrToLOG(debug_text, wait=True)
            if moveWithBL:
                # calculate the delta to move
                # The calculated delta is the scaled, and used for both absolute and relative
                # movements
                delta = motorEndPos - motorStartPos
                debug_text = f"{tc_no}#{lineno()} delta={delta}"
                self.axisCom.putDbgStrToLOG(debug_text, wait=True)
                if cnt > 1:
                    if rmod == motorRMOD_A:
                        delta = delta * factor
                    elif rmod == motorRMOD_G:
                        delta = delta * factor

                if encRel:
                    line1 = (
                        "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
                        % (delta * frac, self.myBVEL, self.myBAR, motorStartPos)
                    )
                else:
                    if self.isMotorMasterAxis:
                        deltaFrac = delta * frac
                    else:
                        deltaFrac = delta
                    line1 = (
                        "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
                        % (
                            motorStartPos + deltaFrac,
                            self.myBVEL,
                            self.myBAR,
                            motorStartPos,
                        )
                    )
                    # The way the simulation is written,
                    # the motor/master re-tries and moves from startpos to startpos
                    # But not in the last round
                    if self.isMotorMasterAxis and (cnt < maxcnt - 1):
                        line2 = (
                            "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
                            % (motorStartPos, self.myVELO, self.myAR, motorStartPos)
                        )

                expFile.write(f"{line1}{line2}")
                cnt += 1
            else:
                # As we don't move the motor (it is simulated, we both times start at motorStartPos
                # calculate the delta to move
                # The calculated delta is the scaled, and used for both absolute and relative
                # movements
                deltaPos1 = motorEndPos - motorStartPos - bdst
                deltaToMov1 = deltaPos1 * factor
                delta = motorEndPos - motorStartPos
                debug_text = f"{tc_no}#{lineno()} deltaToMov1={deltaToMov1}"
                self.axisCom.putDbgStrToLOG(debug_text, wait=True)
                if encRel:
                    # Do not move relative 0
                    if deltaToMov1 != 0:
                        line1 = (
                            "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
                            % (deltaToMov1, self.myVELO, self.myAR, motorStartPos)
                        )
                    # Move forward with backlash parameters
                    # Note: This should be bdst, but since we don't move the motor AND
                    # the record uses the readback value, use "motorEndPos - motorStartPos"
                    delta = motorEndPos - motorStartPos
                    deltaToMov2 = delta * frac
                    debug_text = f"{tc_no}#{lineno()} line2 deltaToMov2={deltaToMov2}"
                    self.axisCom.putDbgStrToLOG(debug_text, wait=True)
                    line2 = (
                        "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
                        % (deltaToMov2, self.myBVEL, self.myBAR, motorStartPos)
                    )
                else:
                    # Do not move relative 0
                    if deltaToMov1 != 0:
                        line1 = (
                            "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
                            % (
                                motorStartPos + deltaToMov1,
                                self.myVELO,
                                self.myAR,
                                motorStartPos,
                            )
                        )
                    self.axisCom.putDbgStrToLOG(debug_text, wait=True)
                    # Move forward with backlash parameters
                    # motor/master moves too far after the motor had gone stuck
                    ## motor/master uses frac for backlash. motorRecord uses mres when talking to hardware
                    # double bpos = (pmr->dval - pmr->bdst) / pmr->mres;
                    # double currpos = pmr->dval / pmr->mres;
                    # double newpos = bpos + pmr->frac * (currpos - bpos);
                    bpos = motorEndPos - bdst
                    if self.isMotorMasterAxis:
                        destPos2 = bpos + frac * (motorEndPos - bpos)
                    else:
                        destPos2 = motorEndPos
                    debug_text = (
                        f"{tc_no}#{lineno()} bpos={bpos} motorEndPos={motorEndPos}"
                    )
                    self.axisCom.putDbgStrToLOG(debug_text, wait=True)

                    debug_text = f"{tc_no}#{lineno()} destPos2={destPos2}"
                    self.axisCom.putDbgStrToLOG(debug_text, wait=True)

                    line2 = (
                        "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n"
                        % (destPos2, self.myBVEL, self.myBAR, motorStartPos)
                    )
                if directionOfBL == 0:
                    expFile.write(f"{line1}")
                elif need_007_017_tweak and cnt == 0:
                    # No retry yet
                    expFile.write(f"{line2}")
                else:
                    expFile.write(f"{line1}{line2}")
                cnt += 1
        expFile.write("EOF\n")
        expFile.close()
