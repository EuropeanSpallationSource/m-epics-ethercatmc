#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisCom import AxisCom


filnam = os.path.basename(__file__)[0:3]
###


polltime = 0.1


def lineno():
    return inspect.currentframe().f_back.f_lineno


def readCabinetBitName(self, tc_no, bitNo):
    # IOC:CabinetBitNam0
    ret = repr(self.axisCom.get("CabinetBitNam" + str(bitNo)))
    # Remove the "'" at the begin and end which we get from repr()
    # which we need because I couldn't figure out a way to get a string
    # without the timestamp
    ret = ret.strip("'").rstrip("'")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} readCabinetBitName bitNo={bitNo} ret={ret}"
    )
    return ret


def checkCabinetBitName(self, tc_no, bitNo, exp):
    act = cabinetBitNames[bitNo]
    passed = exp == act
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} exp={exp} act={act} passed={passed}"
    )
    return passed


cabinetBitNames = {}


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    afterLastColon = 1 + url_string.rindex(":")
    cpu_string = url_string[:afterLastColon]
    mtr_str = url_string[afterLastColon:]
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string} afterLastColon={afterLastColon} cpu_string={cpu_string} mtr_str={mtr_str}"
    )
    # The url "points to" a motor, we need the PV name without the motor
    axisCom = AxisCom(cpu_string, log_debug=False)
    # Note: No AxisMr, we do not have an axis
    tc_no = int(filnam) * 1000
    # Normally we test a motor -  and that has a the motor has a
    # axisCom.putDbgStrToLOG() method. However, we do not test the motor
    # But a different PV.
    # For logging into the IOC log we borrow the motor.
    # This is probably OK for a simulator test
    axisCom.put(mtr_str + "-DbgStrToLOG", "Start " + str(tc_no), wait=True)
    passed = True
    if passed:
        axisCom.put(mtr_str + "-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
    else:
        axisCom.put(mtr_str + "-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
    assert passed

    # Read all the bit names
    def test_TC_942001(self):
        tc_no = "TC-942001"
        mtr_str = self.mtr_str
        self.axisCom.put(mtr_str + "-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        bitNo = 0
        while bitNo < 24:
            bitName = readCabinetBitName(self, tc_no, bitNo)
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} bitNo={bitNo} bitName={bitName}"
            )
            cabinetBitNames[bitNo] = bitName
            if len(bitName) > 0:
                passed = True
            bitNo += 1

        if passed:
            self.axisCom.put(
                mtr_str + "-DbgStrToLOG", "Passed " + str(tc_no), wait=True
            )
        else:
            self.axisCom.put(
                mtr_str + "-DbgStrToLOG", "Failed " + str(tc_no), wait=True
            )
        assert passed

    # Check all the bits one by one
    def test_TC_942002(self):
        tc_no = "TC-942002"
        mtr_str = self.mtr_str
        self.axisCom.put(mtr_str + "-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        passed = True
        passed = passed and checkCabinetBitName(self, tc_no, 0, "24VPSFailed")
        passed = passed and checkCabinetBitName(self, tc_no, 1, "48VPSFailed")
        passed = passed and checkCabinetBitName(self, tc_no, 2, "MCBError")
        passed = passed and checkCabinetBitName(self, tc_no, 3, "SPDError")
        passed = passed and checkCabinetBitName(self, tc_no, 4, "DoorOpen")
        passed = passed and checkCabinetBitName(self, tc_no, 5, "FuseTripped")
        passed = passed and checkCabinetBitName(self, tc_no, 6, "EStop")
        passed = passed and checkCabinetBitName(self, tc_no, 7, "TempHigh")
        passed = passed and checkCabinetBitName(self, tc_no, 8, "ECMasterErr")
        passed = passed and checkCabinetBitName(self, tc_no, 9, "SlaveNotOP")
        passed = passed and checkCabinetBitName(self, tc_no, 10, "SlaveMissing")
        passed = passed and checkCabinetBitName(self, tc_no, 11, "CPULoadHigh")
        passed = passed and checkCabinetBitName(self, tc_no, 12, "")
        passed = passed and checkCabinetBitName(self, tc_no, 13, "")
        passed = passed and checkCabinetBitName(self, tc_no, 14, "")
        passed = passed and checkCabinetBitName(self, tc_no, 15, "")
        passed = passed and checkCabinetBitName(self, tc_no, 16, "")
        passed = passed and checkCabinetBitName(self, tc_no, 17, "")
        passed = passed and checkCabinetBitName(self, tc_no, 18, "")
        passed = passed and checkCabinetBitName(self, tc_no, 19, "")
        passed = passed and checkCabinetBitName(self, tc_no, 20, "")
        passed = passed and checkCabinetBitName(self, tc_no, 21, "")
        passed = passed and checkCabinetBitName(self, tc_no, 22, "")
        passed = passed and checkCabinetBitName(self, tc_no, 23, "")

        if passed:
            self.axisCom.put(
                mtr_str + "-DbgStrToLOG", "Passed " + str(tc_no), wait=True
            )
        else:
            self.axisCom.put(
                mtr_str + "-DbgStrToLOG", "Failed " + str(tc_no), wait=True
            )
        assert passed

    def teardown_class(self):
        tc_no = int(filnam) * 1000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
