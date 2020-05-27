#!/usr/bin/env python

import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

###

PwrOnDly = 6.0
PwrOffDly = 3.0


def restorePwrSettings(self, tc_no, pwrAuto, pwrOnDly, pwrOffDly):
    self.axisCom.put("-PwrAuto", pwrAuto)
    self.axisCom.put("-PwrOnDly", pwrOnDly)
    self.axisCom.put("-PwrOffDly", pwrOffDly)


def do_220_autopower(self, tc_no, autopower):
    self.axisCom.put("-DbgStrToLOG", "Start " + tc_no[0:20])
    self.axisMr.setCNENandWait(tc_no, 0)
    self.axisCom.put("-PwrAuto", autopower)
    self.axisCom.put("-PwrOnDly", PwrOnDly)
    self.axisCom.put("-PwrOffDly", PwrOffDly)
    print(f"{tc_no} Enable move to LLM +10")
    destination = self.saved_LLM + 10 + 2 * autopower
    done = self.axisMr.moveWait(tc_no, destination)

    # Make sure drive is still enabled
    power1 = self.axisCom.get(".CNEN", use_monitor=False)
    print("%s Check drive is still enabled power1=%d" % (tc_no, power1))

    time.sleep(PwrOnDly + PwrOffDly + 2.0)
    power2 = self.axisCom.get(".CNEN", use_monitor=False)
    print("%s Wait 8s and check drive is now disabled power2=%d" % (tc_no, power2))
    restorePwrSettings(
        self, tc_no, self.saved_PwrAuto, self.saved_PwrOnDly, self.saved_PwrOffDly,
    )
    self.axisMr.setCNENandWait(tc_no, self.saved_CNEN)

    print("%s done=%s power1=%d power2=%d" % (tc_no, done, power1, power2))
    if (done == 1) and (power1 == 1) and (power2 == 0):
        testPassed = True
    else:
        testPassed = False

    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    # self.axisCom.put('-DbgStrToLOG', "Start of " + os.path.basename(__file__)[0:20])
    saved_LLM = axisCom.get(".LLM")
    saved_CNEN = axisCom.get(".CNEN")
    saved_PwrAuto = axisCom.get("-PwrAuto")
    saved_PwrOnDly = axisCom.get("-PwrOnDly")
    saved_PwrOffDly = axisCom.get("-PwrOffDly")

    def test_TC_2200(self):
        tc_no = "2201-Enable_goto_LLM"

        # Enable power
        self.axisCom.put("-DbgStrToLOG", "Start " + tc_no[0:20])
        print(f"{tc_no} Enable drive and move to LLM")
        self.axisCom.put("-PwrAuto", 2)
        self.axisCom.put("-PwrOnDly", PwrOnDly)
        self.axisMr.setCNENandWait(tc_no, 1)
        destination = self.saved_LLM
        done = self.axisMr.moveWait(tc_no, destination)
        restorePwrSettings(
            self, tc_no, self.saved_PwrAuto, self.saved_PwrOnDly, self.saved_PwrOffDly,
        )
        self.axisCom.put("-DbgStrToLOG", "End   " + tc_no[0:20])
        self.assertEqual(1, done, "moveWait should return done")

    def test_TC_2201(self):
        tc_no = "2201-Auto_pwr_1"
        print(f"{tc_no} autopower ")
        do_220_autopower(self, tc_no, 1)

    def test_TC_2202(self):
        tc_no = "2202-Auto_pwr_2"
        print(f"{tc_no} autopower ")
        do_220_autopower(self, tc_no, 2)
        self.axisMr.setCNENandWait(tc_no, self.saved_CNEN)
