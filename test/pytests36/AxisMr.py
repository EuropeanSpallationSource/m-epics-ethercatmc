#!/usr/bin/python

"""

"""

import sys
import math
import time
import os
import filecmp

from AxisCom import AxisCom


motorRMOD_D = 0  # "Default"
motorRMOD_A = 1  # "Arithmetic"
motorRMOD_G = 2  # "Geometric"
motorRMOD_I = 3  # "In-Position"


polltime = 0.2


class AxisMr:
    def __init__(self, axisCom, url_string=None):
        self.axisCom = axisCom
        self.url_string = url_string

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

    def initializeMotorRecordOneField(self, tc_no, field_name, value):
        oldVal = self.axisCom.get(field_name)

        if oldVal != None:
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

        # If there are usful values in the controller, use them
        cfgDHLM = self.axisCom.get("-CfgDHLM")
        cfgDLLM = self.axisCom.get("-CfgDLLM")
        if cfgDHLM == None or cfgDLLM == None or cfgDHLM <= cfgDLLM:
            cfgDHLM = 53.0
            cfgDLLM = -54.0
            self.setSoftLimitsOff(tc_no)
            self.setValueOnSimulator(tc_no, "fHighSoftLimitPos", cfgDHLM)
            self.setValueOnSimulator(tc_no, "fLowSoftLimitPos", cfgDLLM)
            self.initializeMotorRecordOneField(tc_no, "-CfgDHLM-En", 1)
            self.initializeMotorRecordOneField(tc_no, "-CfgDLLM-En", 1)
            self.initializeMotorRecordOneField(tc_no, ".DHLM", cfgDHLM)
            self.initializeMotorRecordOneField(tc_no, ".DLLM", cfgDLLM)

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
        # if (msta & self.MSTA_BIT_GAIN_SUPPORT):
        #    ret = ret + 'G'
        # else:
        #    ret = ret +'.'
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
        return ret

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
        return ret

    def calcAlmostEqual(self, tc_no, expected, actual, maxdelta):
        delta = math.fabs(expected - actual)
        delta <= maxdelta
        if delta <= maxdelta:
            inrange = True
        else:
            inrange = False

        print(
            f"{tc_no}: calcAlmostEqual expected={expected} actual={actual} delta={delta} maxdelta={maxdelta} inrange={inrange}"
        )
        return inrange

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
        return timeout

    def waitForStart(self, tc_no, wait_for_start):
        while wait_for_start > 0:
            wait_for_start -= polltime
            dmov = int(self.axisCom.get(".DMOV", use_monitor=False))
            movn = int(self.axisCom.get(".MOVN", use_monitor=False))
            rbv = self.axisCom.get(".RBV")
            debug_text = f"{tc_no}: wait_for_start={wait_for_start} dmov={dmov} movn={movn} rbv={rbv}"
            print(debug_text)
            if movn and not dmov:
                return
            time.sleep(polltime)
            wait_for_start -= polltime
        raise Exception(debug_text)

    def waitForStop(self, tc_no, wait_for_stop):
        while wait_for_stop > 0:
            wait_for_stop -= polltime
            dmov = int(self.axisCom.get(".DMOV", use_monitor=False))
            movn = int(self.axisCom.get(".MOVN", use_monitor=False))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            debug_text = f"{tc_no}: wait_for_stop={wait_for_stop} dmov={dmov} movn={movn} rbv={rbv}"
            print(debug_text)
            if not movn and dmov:
                return
            time.sleep(polltime)
            wait_for_stop -= polltime
        raise Exception(debug_text)

    def waitForStartAndDone(self, tc_no, wait_for_done):
        wait_for_start = 2
        while wait_for_start > 0:
            wait_for_start -= polltime
            dmov = int(self.axisCom.get(".DMOV"))
            movn = int(self.axisCom.get(".MOVN"))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            debug_text = f"{tc_no}: wait_for_start_and_done_start={wait_for_start} dmov={dmov} movn={movn} rbv={rbv}"
            print(debug_text)
            if movn or not dmov:
                wait_for_start = 0
            else:
                time.sleep(polltime)

        wait_for_done = math.fabs(wait_for_done)  # negative becomes positive
        wait_for_done += 1  # One extra second for rounding
        while wait_for_done > 0:
            dmov = int(self.axisCom.get(".DMOV"))
            movn = int(self.axisCom.get(".MOVN"))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            debug_text = f"{tc_no}: wait_for_start_and_done_done={wait_for_done} dmov={dmov} movn={movn} rbv={rbv}"
            print(debug_text)
            if dmov and not movn:
                return
            time.sleep(polltime)
            wait_for_done = wait_for_done - polltime
        raise Exception(debug_text)

    def waitForMipZero(self, tc_no, wait_for_mip_zero):
        while wait_for_mip_zero > -10.0:  # Extra long wait
            wait_for_mip_zero -= polltime
            mip = int(self.axisCom.get(".MIP", use_monitor=False))
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            debug_text = f"{tc_no}: wait_for_mip_zero={wait_for_mip_zero} mip={self.getMIPtext(mip)} (0x{mip:04x}) rbv ={rbv }"
            print(debug_text)
            if not mip:
                return
            time.sleep(polltime)
            wait_for_mip_zero -= polltime
        raise Exception(debug_text)

    def waitForPowerOn(self, tc_no, wait_for_powerOn):
        while wait_for_powerOn > 0:
            wait_for_powerOn -= polltime
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            powerOn = msta & self.MSTA_BIT_AMPON
            debug_text = (
                f"{tc_no}: wait_for_powerOn={wait_for_powerOn} powerOn={int(powerOn)}"
            )
            print(debug_text)
            if powerOn:
                return
            time.sleep(polltime)
            wait_for_powerOn -= polltime
        raise Exception(debug_text)

    def waitForPowerOff(self, tc_no, wait_for_powerOff):
        while wait_for_powerOff > 0:
            wait_for_powerOff -= polltime
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            powerOn = msta & self.MSTA_BIT_AMPON
            debug_text = (
                f"{tc_no}: wait_for_powerOff={wait_for_powerOff} powerOn={int(powerOn)}"
            )
            print(debug_text)
            if not powerOn:
                return True
            time.sleep(polltime)
            wait_for_powerOff -= polltime
        raise Exception(debug_text)

    def jogDirection(self, tc_no, direction):
        jvel = self.axisCom.get(".JVEL")
        hlm = self.axisCom.get(".HLM")
        llm = self.axisCom.get(".LLM")
        rbv = self.axisCom.get(".RBV")
        accl = self.axisCom.get(".ACCL")
        deltah = math.fabs(hlm - rbv)
        deltal = math.fabs(llm - rbv)
        # TODO: we could use at the DIR field, which delta to use
        # This can be done in a cleanup
        if deltah > deltal:
            delta = deltah
        else:
            delta = deltal
        # TODO: add JAR to the calculation
        time_to_wait = delta / jvel + 2 * accl + 2.0
        if direction > 0:
            self.axisCom.put(".JOGF", 1)
        else:
            self.axisCom.put(".JOGR", 1)

        self.waitForStartAndDone(tc_no + " jogDirection", 30 + time_to_wait + 3.0)

        if direction > 0:
            self.axisCom.put(".JOGF", 0)
        else:
            self.axisCom.put(".JOGR", 0)

    #    def movePosition(self, tc_no, destination, velocity, acceleration):
    #        time_to_wait = 30
    #        if velocity > 0:
    #            distance = math.fabs(self.axisCom.get(".RBV") - destination)
    #            time_to_wait += distance / velocity + 2 * acceleration
    #        self.axisCom.put(".VAL", destination)
    #        done = self.waitForStartAndDone(tc_no + " movePosition", time_to_wait)

    def moveWait(self, tc_no, destination):
        timeout = 30
        acceleration = self.axisCom.get(".ACCL")
        velocity = self.axisCom.get(".VELO")
        timeout += 2 * acceleration + 1.0
        if velocity > 0:
            distance = math.fabs(self.axisCom.get(".RBV") - destination)
            timeout += distance / velocity

        self.axisCom.put(".VAL", destination)
        self.waitForStartAndDone(str(tc_no) + " movePosition", timeout)

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
                    "%s/%s: setValueOnSimulator lenOutStr=%d outStr=%s"
                    % (tc_no, self - url_string, lenOutStr, outStr)
                )
                assert len(outStr) < 40
        self.axisCom.put("-DbgStrToMCU", outStr, wait=True)
        err = int(self.axisCom.get("-Err", use_monitor=False))
        print(
            "%s/%s: DbgStrToMCU  var=%s value=%s err=%d"
            % (tc_no, self.url_string, var, value, err)
        )
        assert not err

    def motorInitAllForBDST(self, tc_no):
        self.setValueOnSimulator(tc_no, "nAmplifierPercent", 100)
        self.setValueOnSimulator(tc_no, "bAxisHomed", 1)
        self.setValueOnSimulator(tc_no, "fLowHardLimitPos", -100)
        self.setValueOnSimulator(tc_no, "fHighHardLimitPos", 100)
        self.setValueOnSimulator(tc_no, "setMRES_23", 0)
        self.setValueOnSimulator(tc_no, "setMRES_24", 0)

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

    def writeExpFileRMOD_X(
        self, tc_no, rmod, expFile, maxcnt, frac, encRel, motorStartPos, motorEndPos,
    ):
        cnt = 0
        if motorEndPos - motorStartPos > 0:
            directionOfMove = 1
        else:
            directionOfMove = -1
        if self.myBDST > 0:
            directionOfBL = 1
        else:
            directionOfBL = -1

        print(
            f"{tc_no}: writeExpFileRMOD_X encRel={encRel} motorStartPos={motorStartPos} motorEndPos={motorEndPos} directionOfMove{directionOfMove} directionOfBL={directionOfBL}"
        )

        if rmod == motorRMOD_I:
            maxcnt = 1  # motorRMOD_I means effecttivly "no retry"
            encRel = 0

        if (
            abs(motorEndPos - motorStartPos) <= abs(self.myBDST)
            and directionOfMove == directionOfBL
        ):
            while cnt < maxcnt:
                # calculate the delta to move
                # The calculated delta is the scaled, and used for both absolute and relative
                # movements
                delta = motorEndPos - motorStartPos
                if cnt > 1:
                    if rmod == motorRMOD_A:
                        # From motorRecord.cc:
                        # factor = (pmr->rtry - pmr->rcnt + 1.0) / pmr->rtry;
                        factor = 1.0 * (self.myRTRY - cnt + 1.0) / self.myRTRY
                        delta = delta * factor
                    elif rmod == motorRMOD_G:
                        # factor = 1 / pow(2.0, (pmr->rcnt - 1));
                        rcnt_1 = cnt - 1
                        factor = 1.0
                        while rcnt_1 > 0:
                            factor = factor / 2.0
                            rcnt_1 -= 1
                        delta = delta * factor

                if encRel:
                    line1 = (
                        "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                        % (delta * frac, self.myBVEL, self.myBAR, motorStartPos)
                    )
                else:
                    line1 = (
                        "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                        % (
                            motorStartPos + delta,
                            self.myBVEL,
                            self.myBAR,
                            motorStartPos,
                        )
                    )
                expFile.write(f"{line1}\n")
                cnt += 1
        else:
            # As we don't move the motor (it is simulated, we both times start at motorStartPos
            while cnt < maxcnt:
                # calculate the delta to move
                # The calculated delta is the scaled, and used for both absolute and relative
                # movements
                delta = motorEndPos - motorStartPos - self.myBDST
                if cnt > 1:
                    if rmod == motorRMOD_A:
                        # From motorRecord.cc:
                        # factor = (pmr->rtry - pmr->rcnt + 1.0) / pmr->rtry;
                        factor = 1.0 * (self.myRTRY - cnt + 1.0) / self.myRTRY
                        delta = delta * factor
                    elif rmod == motorRMOD_G:
                        # factor = 1 / pow(2.0, (pmr->rcnt - 1));
                        rcnt_1 = cnt - 1
                        factor = 1.0
                        while rcnt_1 > 0:
                            factor = factor / 2.0
                            rcnt_1 -= 1
                        delta = delta * factor

                if encRel:
                    line1 = (
                        "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                        % (delta * frac, self.myVELO, self.myAR, motorStartPos)
                    )
                    # Move forward with backlash parameters
                    # Note: This should be self.myBDST, but since we don't move the motor AND
                    # the record uses the readback value, use "motorEndPos - motorStartPos"
                    delta = motorEndPos - motorStartPos
                    line2 = (
                        "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                        % (delta * frac, self.myBVEL, self.myBAR, motorStartPos)
                    )
                else:
                    line1 = (
                        "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                        % (motorStartPos + delta, self.myVELO, self.myAR, motorStartPos)
                    )
                    # Move forward with backlash parameters
                    line2 = (
                        "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                        % (motorEndPos, self.myBVEL, self.myBAR, motorStartPos)
                    )

                expFile.write(f"{line1}\n{line2}\n")
                cnt += 1
        expFile.write("EOF\n")
        expFile.close()

    def writeExpFileJOG_BDST(
        self, tc_no, expFileName, myDirection, frac, encRel, motorStartPos, motorEndPos,
    ):
        # Create a "expected" file
        expFile = open(expFileName, "w")

        # The jogging command
        line1 = (
            "move velocity direction=%d max_velocity=%g acceleration=%g motorPosNow=%g"
            % (myDirection, self.myJVEL, self.myJAR, motorStartPos)
        )
        deltaForth = self.myBDST * frac
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
            line3 = (
                "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                % (deltaBack, self.myBVEL, self.myBAR, motorEndPos - deltaForth)
            )
        else:
            # Move back in positioning mode
            line2 = (
                "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                % (motorEndPos - self.myBDST, self.myVELO, self.myAR, motorEndPos)
            )
            # Move forward with backlash parameters
            line3 = (
                "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g"
                % (motorEndPos, self.myBVEL, self.myBAR, motorEndPos - self.myBDST)
            )

        expFile.write(f"{line1}\n{line2}\n{line3}\nEOF\n")
        expFile.close()

    def cmpUnlinkExpectedActualFile(self, tc_no, expFileName, actFileName):
        # compare actual and expFile
        sameContent = False
        wait_for_found = 5
        while wait_for_found > 0:
            try:
                file = open(expFileName)
                for line in file:
                    if line[-1] == "\n":
                        line = line[0:-1]
                    print(f"{expFileName}: {str(line)}")
                file.close()
                file = open(actFileName)
                for line in file:
                    if line[-1] == "\n":
                        line = line[0:-1]
                    print(f"{actFileName}: {str(line)}")
                file.close()
                sameContent = filecmp.cmp(expFileName, actFileName, shallow=False)
                wait_for_found = 0
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

            wait_for_found -= polltime

        return sameContent

    def setSoftLimitsOff(self, tc_no):
        """
        Switch off the soft limits
        """
        actDHLM = self.axisCom.get(".DHLM", use_monitor=False)
        actDLLM = self.axisCom.get(".DLLM", use_monitor=False)
        print(f"{tc_no}: setSoftLimitsOff hlm={actDHLM} llm={actDLLM}")
        # switch off the controller soft limits
        self.axisCom.put("-CfgDHLM-En", 0, wait=True)
        self.axisCom.put("-CfgDLLM-En", 0, wait=True)

        maxTime = 10  # seconds maximum to let read only parameters ripple through
        maxDelta = 0.05  # 5 % error tolerance margin
        while maxTime > 0:
            self.axisCom.put(".DLLM", 0.0)
            self.axisCom.put(".DHLM", 0.0)

            actDHLM = self.axisCom.get(".DHLM", use_monitor=False)
            actDLLM = self.axisCom.get(".DLLM", use_monitor=False)

            print(f"{tc_no}: setSoftLimitsOff dhlm={actDHLM} dllm={actDLLM}")
            resH = self.calcAlmostEqual(tc_no, 0.0, actDHLM, maxDelta)
            resL = self.calcAlmostEqual(tc_no, 0.0, actDLLM, maxDelta)
            debug_text = f"{tc_no}: setSoftLimitsOff actDHLM={actDHLM} actDLLM={actDLLM} resH={resH} resL={resL}"
            print(debug_text)
            if (resH == True) and (resL == True):
                return

            time.sleep(polltime)
            maxTime = maxTime - polltime
        raise Exception(debug_text)
        assert False

    def setSoftLimitsOn(self, low_limit, high_limit):
        """
        Set the soft limits
        """
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

    def doSTUPandSYNC(self, tc_no):
        self.waitForMipZero(tc_no, 2)
        stup = self.axisCom.get(".STUP", use_monitor=False)
        while stup != 0:
            stup = self.axisCom.get(".STUP", use_monitor=False)
            print(f"{tc_no} doSTUPandSYNC .STUP={stup}")
            time.sleep(polltime)

        self.axisCom.put(".STUP", 1)
        self.axisCom.put(".SYNC", 1)
        self.waitForMipZero(tc_no, 2)
        rbv = self.axisCom.get(".RBV", use_monitor=False)
        print(f"{tc_no} doSTUPandSYNC .RBV={rbv:f} .STUP={stup}")
        while stup != 0:
            stup = self.axisCom.get(".STUP", use_monitor=False)
            rbv = self.axisCom.get(".RBV", use_monitor=False)
            print(f"{tc_no} doSTUPandSYNC.RBV={rbv:f} .STUP={stup}")
            time.sleep(polltime)
        self.waitForMipZero(tc_no, 2)
        msta = int(self.axisCom.get(".MSTA", use_monitor=False))
        print(f"{tc_no} doSTUPandSYNC msta={self.getMSTAtext(msta)}")

    def setCNENandWait(self, tc_no, cnen):
        wait_for_power_changed = 6.0
        # capv_self.axisMr.capvput(
        #     + "-DbgStrToLOG", "CNEN=" + str(cnen) + " " + tc_no[0:20]
        # )
        self.axisCom.put(".CNEN", cnen)
        while wait_for_power_changed > 0:
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            print(
                "%s: wait_for_power_changed=%f msta=%x %s"
                % (tc_no, wait_for_power_changed, msta, self.getMSTAtext(msta))
            )
            if cnen and (msta & self.MSTA_BIT_AMPON):
                return True
            if not cnen and not (msta & self.MSTA_BIT_AMPON):
                return True
            time.sleep(polltime)
            wait_for_power_changed -= polltime
        return False

    def resetAxis(self, tc_no):
        wait_for_ErrRst = 5
        err = int(self.axisCom.get("-Err", use_monitor=False))
        print(f"{tc_no} resetAxis err={int(err)}")

        self.axisCom.put("-ErrRst", 1)
        while wait_for_ErrRst > 0:
            wait_for_ErrRst -= polltime
            err = int(self.axisCom.get("-Err", use_monitor=False))
            print(f"{tc_no} wait_for_ErrRst={wait_for_ErrRst:f} err=0x{err:x}")
            if not err:
                return True
            time.sleep(polltime)
            wait_for_ErrRst -= polltime
        return False

    def verifyRBVinsideRDBD(self, tc_no, position):
        """

        """
        rdbd = self.axisCom.get(".RDBD")
        rbv = self.axisCom.get(".RBV")

        if (rbv < position - rdbd) or (rbv > position + rdbd):
            print(
                "%s: verifyRBVinsideRDBD position=%f rbv=%f rdbd=%f"
                % (tc_no, position, rbv, rdbd)
            )
            return False
        return True
