#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#

import epics
import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
from motor_globals import motor_globals
###

PwrOnDly  =  6.0
PwrOffDly =  3.0


def restorePwrSettings(self, motor, tc_no, pwrAuto, pwrOnDly, pwrOffDly):
    epics.caput(motor + '-PwrAuto', pwrAuto)
    epics.caput(motor + '-PwrOnDly', pwrOnDly)
    epics.caput(motor + '-PwrOffDly', pwrOffDly)

def do_220_autopower(self, motor, tc_no, autopower):
    self.lib.setCNENandWait(motor, tc_no, 0)
    epics.caput(motor + '-PwrAuto', autopower, wait=True, timeout=self.g.TIMEOUT)
    epics.caput(motor + '-PwrOnDly', PwrOnDly, wait=True, timeout=self.g.TIMEOUT)
    epics.caput(motor + '-PwrOffDly', PwrOffDly, wait=True, timeout=self.g.TIMEOUT)
    print('%s Enable move to LLM +10' % tc_no)
    res1 = self.lib.move(motor, self.saved_LLM + 10 + 2*autopower, self.g.TIMEOUT)

    #Make sure drive is still enabled
    power1 = epics.caget(motor + '.CNEN', use_monitor=False)
    print('%s Check drive is still enabled power1=%d' % (tc_no, power1))


    time.sleep(PwrOnDly + PwrOffDly + 2.0)
    power2 = epics.caget(motor + '.CNEN', use_monitor=False)
    print('%s Wait 8s and check drive is now disabled power2=%d' % (tc_no, power2))
    restorePwrSettings(self, motor, tc_no, self.saved_PwrAuto, self.saved_PwrOnDly, self.saved_PwrOffDly)
    assert(res1 == 0)
    assert(power1 == 1)
    assert(power2 == 0)



class Test(unittest.TestCase):
    lib = motor_lib()
    g = motor_globals()
    motor           = os.getenv("TESTEDMOTORAXIS")
    epics.caput(motor + '-DbgStrToLOG', "Start of " + os.path.basename(__file__)[0:20])
    pv_Err          = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-Err")
    pv_nErrorId     = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrId")
    pv_nErrRst      = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrRst")
    saved_LLM       = epics.caget(motor + '.LLM')
    saved_CNEN      = epics.caget(motor + '.CNEN')
    saved_PwrAuto   = epics.caget(motor + '-PwrAuto')
    saved_PwrOnDly  = epics.caget(motor + '-PwrOnDly')
    saved_PwrOffDly = epics.caget(motor + '-PwrOffDly')


    def test_TC_2200(self):
        motor = self.motor
        tc_no = "TC-2201-Enable_goto_LLM"

        #Enable power
        print('%s Enable drive and move to LLM' % tc_no)
        epics.caput(motor + '-PwrAuto', 2,         wait=True, timeout=self.g.TIMEOUT)
        epics.caput(motor + '-PwrOnDly', PwrOnDly, wait=True, timeout=self.g.TIMEOUT)
        self.lib.setCNENandWait(motor, tc_no, 1)
        res = self.lib.move(motor, self.saved_LLM, self.g.TIMEOUT)
        restorePwrSettings(self, motor, tc_no, self.saved_PwrAuto, self.saved_PwrOnDly, self.saved_PwrOffDly)
        assert(res == 0)


    def test_TC_2201(self):
        motor = self.motor
        tc_no = "TC-2201-Auto_power_mode_1"
        print('%s autopower ' % tc_no)
        do_220_autopower(self, motor, tc_no, 1)

    def test_TC_2202(self):
        motor = self.motor
        tc_no = "TC-2202-Auto_power_mode_2"
        print('%s autopower ' % tc_no)
        do_220_autopower(self, motor, tc_no, 2)
        self.lib.setCNENandWait(motor, tc_no, self.saved_CNEN)
