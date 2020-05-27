import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math
import inspect

###


def lineno():
    return inspect.currentframe().f_back.f_lineno


# Amplifier is locked to be off, but responds "OK".
# See test/simulator/EtherCAT/hw_motor.h
AMPLIFIER_LOCKED_TO_BE_OFF_SILENT = 1


def setValueOnSimulator(self, tc_no, var, value):
    # Note: The driver will change Sim.this.XXX into
    # Sim.M1.XXX
    var = str(var)
    value = str(value)
    outStr = "Sim.this." + var + "=" + value
    print(f"{tc_no}: DbgStrToMCU  var={var} value={var} outStr={outStr}")
    assert len(outStr) < 40
    self.axisCom.put("-DbgStrToMCU", outStr)


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    axisCom.put("-DbgStrToLOG", "Start " + os.path.basename(__file__)[0:20])
    saved_CNEN = axisCom.get(".CNEN")
    saved_PwrAuto = axisCom.get("-PwrAuto")

    # Jog, wait for start, power off, check error, reset error
    def test_TC_9402(self):
        tc_no = "TC-9402-EnabledFailed"

        self.axisCom.put(".CNEN", 0, wait=True)
        setValueOnSimulator(
            self, tc_no, "bAmplifierLockedToBeOff", AMPLIFIER_LOCKED_TO_BE_OFF_SILENT,
        )
        self.axisCom.put("-PwrAuto", 0)
        time.sleep(1.0)

        self.axisCom.put(".CNEN", 1, wait=True)
        time.sleep(4.0)
        mstaErr = int(self.axisCom.get(".MSTA", use_monitor=False))
        print(f"{tc_no} Error mstaErr={self.axisMr.getMSTAtext(mstaErr)}")
        self.axisMr.resetAxis(tc_no)

        setValueOnSimulator(self, tc_no, "bAmplifierLockedToBeOff", 0)

        mstaOKagain = int(self.axisCom.get(".MSTA", use_monitor=False))
        bError = self.axisCom.get("-Err", use_monitor=False)
        nErrorId = self.axisCom.get("-ErrId", use_monitor=False)
        print(
            "%s Clean self.axisMr.MSTA_BIT_PROBLEM=%x mstaOKagain=%s bError=%d nErrorId=%d"
            % (
                tc_no,
                self.axisMr.MSTA_BIT_PROBLEM,
                self.axisMr.getMSTAtext(mstaOKagain),
                bError,
                nErrorId,
            )
        )

        self.axisCom.put(".CNEN", self.saved_CNEN)
        self.axisCom.put("-PwrAuto", self.saved_PwrAuto)

        self.assertNotEqual(
            0,
            mstaErr & self.axisMr.MSTA_BIT_PROBLEM,
            "Error MSTA.Problem should be set)",
        )
        self.assertEqual(
            0,
            mstaErr & self.axisMr.MSTA_BIT_SLIP_STALL,
            "Error MSTA.Slip stall Error should not be set)",
        )
        self.assertEqual(0, mstaErr & self.axisMr.MSTA_BIT_MOVING, "Error MSTA.Moving)")

        self.assertEqual(
            0, mstaOKagain & self.axisMr.MSTA_BIT_MOVING, "Clean MSTA.Moving)"
        )
        self.assertEqual(0, bError, "bError")
        self.assertEqual(0, nErrorId, "nErrorId")
