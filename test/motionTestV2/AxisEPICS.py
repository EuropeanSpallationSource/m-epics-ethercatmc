#
# Class to talk to one "axis"
#
# The "axis" has a "connecton", that can communicate either
# - via EPICS channel access using pyepics
# - via pvacces access using p4p
# - via ADS using pyads
#

import datetime
import inspect
import re
import sys
import time

# import AxisComm
from AxisComm import AxisComm

polltime = 0.2
filnam = "AxisEPICS"

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


def lineno():
    return inspect.currentframe().f_back.f_lineno


class AxisEPICS(AxisComm):
    def __init__(self, conn, pvpfx, log_debug=False):
        self.conn = conn
        self.pvpfx = pvpfx
        self.log_debug = log_debug

    def putDbgStrToLOG(self, value, wait=True, timeout=5.0):
        pvsuf = "-DbgStrToLOG"
        try:
            self.conn.put(self.pvpfx + pvsuf, str(value), wait=wait, timeout=timeout)
        except Exception as ex:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {pvsuf} value='{value}' ex={ex}"
            )

    #    def getMotorPvName(self):
    #        return self.pvpfx

    def disableAxis(self):
        self.conn.put(self.pvpfx + ".CNEN", 0)

    def getAxisTargetPositionWindow(self):
        return self.conn.get(self.pvpfx + ".RDBD", use_monitor=False)

    def getEnabledStatus(self, tc_no=0):
        # cnen = int(self.conn.get(self.pvpfx + ".CNEN"))
        cnen = int(self.conn.get(self.pvpfx + ".CNEN"))
        if cnen == 1:
            return True
        else:
            return False

    def getLimitFwd(self):
        msta = int(self.conn.get(self.pvpfx + ".MSTA", timeout=5))
        if msta & MSTA_BIT_PLUS_LS:
            return True
        else:
            return False

    def getLimitBwd(self):
        msta = int(self.conn.get(self.pvpfx + ".MSTA", timeout=5))
        if msta & MSTA_BIT_MINUS_LS:
            return True
        else:
            return False

    def getActPos(self, tc_no=0):
        return self.conn.get(self.pvpfx + ".RBV", use_monitor=False)

    def getBusyStatus(self, tc_no=0):
        if int(self.conn.get(self.pvpfx + ".MOVN", use_monitor=False)) == 0:
            return False
        else:
            return True

    def getDoneStatus(self, tc_no=0):
        if int(self.conn.get(self.pvpfx + ".DMOV", use_monitor=False)) == 0:
            return False
        else:
            return True

    def getSoftLimitFwdValue(self):
        old_DHLM = self.conn.get(self.pvpfx + ".DHLM")
        old_DLLM = self.conn.get(self.pvpfx + ".DLLM")
        if old_DHLM == 0.0 and old_DLLM == 0.0:
            old_DHLM = self.conn.get(self.pvpfx + "-CfgDHLM-RB")
        return old_DHLM

    def getSoftLimitBwdValue(self):
        old_DHLM = self.conn.get(self.pvpfx + ".DHLM")
        old_DLLM = self.conn.get(self.pvpfx + ".DLLM")
        if old_DHLM == 0.0 and old_DLLM == 0.0:
            old_DLLM = self.conn.get(self.pvpfx + "-CfgDLLM-RB")
        return old_DLLM

    def haltAxis(self):
        self.conn.put(self.pvpfx + ".STOP", 1)

    def setSoftLimitsOff(self, tc_no=0, direction=-1):
        fName = "setSoftLimitsOff"
        """
        Switch off the soft limits
        """
        actDHLM = self.conn.get(self.pvpfx + ".DHLM", use_monitor=False)
        actDLLM = self.conn.get(self.pvpfx + ".DLLM", use_monitor=False)

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} setSoftLimitsOff direction={direction} hlm={actDHLM} llm={actDLLM}"
        )
        # switch off the controller soft limits
        if direction == 0:
            self.conn.put(self.pvpfx + "-CfgDLLM-En", int(0), wait=True)
            self.conn.put(self.pvpfx + "-CfgDHLM-En", int(0), wait=True)
        else:
            self.conn.put(self.pvpfx + "-CfgDHLM-En", int(0), wait=True)
            self.conn.put(self.pvpfx + "-CfgDLLM-En", int(0), wait=True)

        maxTime = 10  # seconds maximum to let read only parameters ripple through
        maxDelta = 0.05  # 5 % error tolerance margin
        while maxTime > 0:
            self.conn.put(self.pvpfx + ".DLLM", 0.0)
            self.conn.put(self.pvpfx + ".DHLM", 0.0)

            actDHLM = self.conn.get(self.pvpfx + ".DHLM", use_monitor=False)
            actDLLM = self.conn.get(self.pvpfx + ".DLLM", use_monitor=False)

            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} setSoftLimitsOff hlm={actDHLM} llm={actDLLM}"
            )
            resH = self.calcAlmostEqual(0.0, actDHLM, maxDelta, tc_no=tc_no)
            resL = self.calcAlmostEqual(0.0, actDLLM, maxDelta, tc_no=tc_no)
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} setSoftLimitsOff hlm={actDHLM} llm={actDLLM} resH={resH} resL={resL}"
            print(debug_text)
            if (resH == True) and (resL == True):
                return

            time.sleep(polltime)
            maxTime = maxTime - polltime
        raise Exception(debug_text)
        assert False

    def setCNENandWait(self, tc_no, cnen):
        fName = "setCNENandWait"
        wait_for = 6.0
        self.conn.put(self.pvpfx + ".CNEN", cnen)
        while wait_for > 0:
            msta = int(self.conn.get(self.pvpfx + ".MSTA", use_monitor=False))
            debug_text = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} wait_for={wait_for:.2f} msta={msta:04x}"

            print(debug_text)
            if cnen and (msta & MSTA_BIT_AMPON):
                return
            if not cnen and not (msta & MSTA_BIT_AMPON):
                return
            time.sleep(polltime)
            wait_for -= polltime
        raise Exception(debug_text)

    def setSoftLimitsOn(
        self, tc_no=0, low_limit=0.0, high_limit=0.0, initAbsMinMax=False
    ):

        fName = "setSoftLimitsOn"
        """
        Set the soft limits
        """
        if initAbsMinMax:
            high_limit = self.conn.get(self.pvpfx + "-CfgPMAX-RB")
            low_limit = self.conn.get(self.pvpfx + "-CfgPMIN-RB")

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} low_limit={low_limit} high_limit={high_limit} initAbsMinMax={initAbsMinMax}"
        )
        # switch on the controller soft limits
        try:
            self.conn.put(
                self.pvpfx + "-CfgDHLM", float(high_limit), wait=True, timeout=2
            )
            self.conn.put(
                self.pvpfx + "-CfgDLLM", float(low_limit), wait=True, timeout=2
            )
            self.conn.put(self.pvpfx + "-CfgDHLM-En", int(1), wait=True, timeout=2)
            self.conn.put(self.pvpfx + "-CfgDLLM-En", int(1), wait=True, timeout=2)
        finally:
            oldRBV = self.conn.get(self.pvpfx + ".RBV")

        if oldRBV < 0:
            self.conn.put(self.pvpfx + ".LLM", low_limit)
            self.conn.put(self.pvpfx + ".HLM", high_limit)
        else:
            self.conn.put(self.pvpfx + ".HLM", high_limit)
            self.conn.put(self.pvpfx + ".LLM", low_limit)

    def enableAxis(self, tc_no=0):
        self.setCNENandWait(tc_no, 1)

    def resetAxis(self, tc_no=0):
        fName = "resetAxis"
        wait_for_ErrRst = 5
        err = int(self.conn.get(self.pvpfx + "-Err", use_monitor=False))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} resetAxis err={int(err)}"
        )

        self.conn.put(self.pvpfx + "-ErrRst", 1)
        while wait_for_ErrRst > 0:
            wait_for_ErrRst -= polltime
            err = int(self.conn.get(self.pvpfx + "-Err", use_monitor=False))
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} wait_for_ErrRst={wait_for_ErrRst:.2f} err=0X{err:X}"
            )
            if not err:
                return True
            time.sleep(polltime)
            wait_for_ErrRst -= polltime
        return False

    def moveAbsolute(self, position):
        self.conn.put(self.pvpfx + ".VAL", position)
