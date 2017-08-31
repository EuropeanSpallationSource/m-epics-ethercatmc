#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#
# m-epics-singlemotion/src/main/test/singlemotion_test.py
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
###

def homeTheMotor(tself, motor, tc_no, procHome, jogToLSBefore):
    old_high_limit = epics.caget(motor + '.HLM')
    old_low_limit = epics.caget(motor + '.LLM')
    old_ProcHom   = tself.pv_ProcHom.get(use_monitor=False)
    if jogToLSBefore != 0:
        tself.lib.setSoftLimitsOff(motor)
        # soft limit range assumed to be = hard range /1.5
        jvel = epics.caget(motor + '.JVEL')
        accl = epics.caget(motor + '.ACCL')
        time_to_wait = 1.5 * (old_high_limit - old_low_limit) / jvel + 2 * accl

        tself.lib.jogDirection(motor, tc_no, jogToLSBefore, time_to_wait)
        epics.caput(motor + '.LLM', old_low_limit)
        epics.caput(motor + '.HLM', old_high_limit)
    else:
        tself.lib.movePosition(motor, tc_no, (
            old_high_limit + old_low_limit) / 2.0, tself.moving_velocity, tself.acceleration)

    tself.pv_ProcHom.put(procHome)
    msta = int(epics.caget(motor + '.MSTA'))
    # We can home while sitting on a limit switch
    if (msta & tself.lib.MSTA_BIT_MINUS_LS):
        epics.caput(motor + '.HOMR', 1)
    else:
        epics.caput(motor + '.HOMF', 1)

    time_to_wait = 180
    started = tself.lib.waitForStart(motor, tc_no + " homeTheMotor", 3)
    msta1 = int(epics.caget(motor + '.MSTA'))
    if (msta1 & tself.lib.MSTA_BIT_HOMED):
      unhomed = 0
    else:
      unhomed = 1
    print '%s homeTheMotor started=%d msta1=%s unhomed=%d' % \
        (tc_no, started, tself.lib.getMSTAtext(msta1), unhomed)
    stopped = tself.lib.waitForStop(motor, tc_no + " homeTheMotor", time_to_wait)

    msta2 = int(epics.caget(motor + '.MSTA'))
    homed = 0
    if (msta2 & tself.lib.MSTA_BIT_HOMED):
        homed = 1
    print '%s homeTheMotor stopped=%d msta2=%s homed=%d' % \
        (tc_no, stopped, tself.lib.getMSTAtext(msta2), homed)
    #tself.assertEqual(True, started,                          tc_no +  "started = True")
    tself.assertEqual(True, stopped,                          tc_no +  "stopped = True")
    tself.assertEqual(0, msta2 & tself.lib.MSTA_BIT_SLIP_STALL, tc_no + "MSTA.no MSTA_BIT_SLIP_STALL")
    tself.assertNotEqual(0, homed,   tc_no + "MSTA.homed (Axis has been homed)")
    tself.pv_ProcHom.put(old_ProcHom, wait=True)

class Test(unittest.TestCase):
    lib = motor_lib()
    m1 = os.getenv("TESTEDMOTORAXIS")
    pv_ProcHom = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ProcHom")

    homing_velocity  = epics.caget(m1 + '.HVEL')
    acceleration     = epics.caget(m1 + '.ACCL')
    moving_velocity  = epics.caget(m1 + '.VELO')
    acceleration     = epics.caget(m1 + '.ACCL')

    def setUp(self):
        print 'set up'

    def tearDown(self):
        print 'clean up'

    # Home against low limit switch
    # Jog against LLS, then home
    def test_TC_11110(self):
        tc_no = "TC-11110"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 1, -1)

    # Move awy from LLS, then home
    def test_TC_11111(self):
        tc_no = "TC-11111"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 1, 0)

    # Home against high limit switch
    # Jog against LLS, then home
    def test_TC_11112(self):
        tc_no = "TC-11112"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 1, 1)

    # Home against high limit switch
    #def test_TC_11120(self):
    #    tc_no = "TC-11120"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 2, 1)

    # Move awy from HLS, then home
    def test_TC_11121(self):
        tc_no = "TC-11121"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 2, 0)

    #def test_TC_11122(self):
    #    tc_no = "TC-11122"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 2, -1)

    # Home against home switch via LLS
    #def test_TC_11130(self):
    #    tc_no = "TC-11130"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 3, 1)

    def test_TC_11131(self):
        tc_no = "TC-11131"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 3, 0)

    #def test_TC_11132(self):
    #    tc_no = "TC-11132"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 3, -1)

    # Home against home switch, via HLS
    #def test_TC_11140(self):
    #    tc_no = "TC-11140"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 4, 1)

    def test_TC_11141(self):
        tc_no = "TC-11141"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 4, 0)

    #def test_TC_11142(self):
    #    tc_no = "TC-11142"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 4, -1)

    # Home against home switch via LLS
    #def test_TC_11150(self):
    #    tc_no = "TC-11150"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 5, 1)

    def test_TC_11151(self):
        tc_no = "TC-11151"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 5, 0)

    #def test_TC_11152(self):
    #    tc_no = "TC-11152"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 5, -1)

    # Home against home switch, via HLS
    #def test_TC_11160(self):
    #    tc_no = "TC-11160"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 6, 1)

    def test_TC_11161(self):
        tc_no = "TC-11161"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 6, 0)

    #def test_TC_11162(self):
    #    tc_no = "TC-11162"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 6, -1)

    # Need to home with the original homing procedure
    def test_TC_11191(self):
        old_ProcHom   = self.pv_ProcHom.get(use_monitor=False)
        tc_no = "TC-11191"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, old_ProcHom, 0)

