#!/usr/bin/python

"""
Author: Matt Pearson
Date: Feb 2015

Description: Class to hold utility functions
"""

import sys
import math
import time

from epics import caput, caget
import epics

from motor_globals import motor_globals

polltime = 0.2

class motor_lib(object):
    """
    Library of useful test functions for motor record applications
    """

    __g = motor_globals()
    MSTA_BIT_HOMED        =  1 << (15 -1)    #4000
    MSTA_BIT_MINUS_LS     =  1 << (14 -1)    #2000
    MSTA_BIT_COMM_ERR     =  1 << (13 -1)    #1000
    MSTA_BIT_GAIN_SUPPORT =  1 << (12 -1)    #0800
    MSTA_BIT_MOVING       =  1 << (11 -1)    #0400
    MSTA_BIT_PROBLEM      =  1 << (10 -1)    #0200
    MSTA_BIT_PRESENT      =  1 << (9 -1)     #0100
    MSTA_BIT_HOME         =  1 << (8 -1)     #0080
    MSTA_BIT_SLIP_STALL   =  1 << (7 -1)     #0040
    MSTA_BIT_AMPON        =  1 << (6 -1)     #0020
    MSTA_BIT_UNUSED       =  1 << (5 -1)     #0010
    MSTA_BIT_HOMELS       =  1 << (4 -1)     #0008
    MSTA_BIT_PLUS_LS      =  1 << (3 -1)     #0004
    MSTA_BIT_DONE         =  1 << (2 -1)     #0002
    MSTA_BIT_DIRECTION    =  1 << (1 -1)     #0001

    def getMSTAtext(msta):
        ret = ''
        if (msta & MSTA_BIT_HOMED):
            ret = ret + 'Hd'
        else:
            ret = ret +'..'
        if (msta & MSTA_BIT_MINUS_LS):
            ret = ret + 'LLS'
        else:
            ret = ret +'...'
        #if (msta & MSTA_BIT_GAIN_SUPPORT):
        #    ret = ret + 'G'
        #else:
        #    ret = ret +'.'
        if (msta & MSTA_BIT_MOVING):
            ret = ret + 'Mov'
        else:
            ret = ret +'...'
        if (msta & MSTA_BIT_PROBLEM):
            ret = ret + 'P'
        else:
            ret = ret +'.'
        if (msta & MSTA_BIT_PRESENT):
            ret = ret + 'Enc'
        else:
            ret = ret +'...'
        if (msta & MSTA_BIT_HOME):
            ret = ret + 'Ho'
        else:
            ret = ret +'..'
        if (msta & MSTA_BIT_SLIP_STALL    ):
            ret = ret + 'Slip'
        else:
            ret = ret +'....'
        if (msta & MSTA_BIT_AMPON):
            ret = ret + 'Amp'
        else:
            ret = ret +'...'
        if (msta & MSTA_BIT_HOMELS):
            ret = ret + 'Hsw'
        else:
            ret = ret +'...'
        if (msta & MSTA_BIT_PLUS_LS):
            ret = ret + 'HLS'
        else:
            ret = ret +'...'
        if (msta & MSTA_BIT_DONE):
            ret = ret + 'Don'
        else:
            ret = ret +'...'
        return ret

    def calcAlmostEqual(self, motor, tc_no, expected, actual, maxdelta):
        delta = math.fabs(expected - actual)
        inrange = delta < maxdelta
        print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (
            tc_no, expected, actual, delta, maxdelta, inrange)
        return inrange

    def waitForStart(self, motor, tc_no, wait_for_start):
        while wait_for_start > 0:
            wait_for_start -= polltime
            dmov = int(epics.caget(motor + '.DMOV', use_monitor=False))
            movn = int(epics.caget(motor + '.MOVN', use_monitor=False))
            rbv = epics.caget(motor + '.RBV')
            print '%s: wait_for_start=%f dmov=%d movn=%d rbv=%f' % (
                tc_no, wait_for_start, dmov, movn, rbv)
            if movn and not dmov:
                return True
            time.sleep(polltime)
            wait_for_start -= polltime
        return False

    def waitForStop(self, motor, tc_no, wait_for_stop):
        while wait_for_stop > 0:
            wait_for_stop -= polltime
            dmov = int(epics.caget(motor + '.DMOV', use_monitor=False))
            movn = int(epics.caget(motor + '.MOVN', use_monitor=False))
            rbv = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s: wait_for_stop=%f dmov=%d movn=%d rbv=%f' % (
                tc_no, wait_for_stop, dmov, movn, rbv)
            if not movn and dmov:
                return True
            time.sleep(polltime)
            wait_for_stop -= polltime
        return False

    def waitForStartAndDone(self, motor, tc_no, wait_for_done):
        wait_for_start = 2
        while wait_for_start > 0:
            wait_for_start -= polltime
            dmov = int(caget(motor + '.DMOV'))
            movn = int(caget(motor + '.MOVN'))
            print '%s: wait_for_start=%f dmov=%d movn=%d dpos=%f' % (
                tc_no, wait_for_start, dmov, movn, caget(motor + '.DRBV', use_monitor=False))
            if movn and not dmov:
               break
            time.sleep(polltime)

        wait_for_done = math.fabs(wait_for_done) #negative becomes positive
        wait_for_done += 1 # One extra second for rounding
        while wait_for_done > 0:
            dmov = int(caget(motor + '.DMOV'))
            movn = int(caget(motor + '.MOVN'))
            print '%s: wait_for_done=%f dmov=%d movn=%d dpos=%f' % (
                tc_no, wait_for_done, dmov, movn, caget(motor + '.DRBV', use_monitor=False))
            if dmov and not movn:
                return True
            time.sleep(polltime)
            wait_for_done -= polltime
        return False


    def testComplete(self, fail):
        """
        Function to be called at end of test
        fail = true or false
        """
        if not fail:
            print "Test Complete"
            return self.__g.SUCCESS
        else:
            print "Test Failed"
            return self.__g.FAIL


    def jogDirection(self, motor, tc_no, direction, time_to_wait):
        if direction > 0:
            epics.caput(motor + '.JOGF', 1)
        else:
            epics.caput(motor + '.JOGR', 1)

        done = waitForStartAndDone(motor, tc_no + " jogDirection", 30 + time_to_wait + 3.0)

        if direction > 0:
            epics.caput(motor + '.JOGF', 0)
        else:
            epics.caput(motor + '.JOGR', 0)

    def move(self, motor, position, timeout):
        """
        Move motor to position. We use put_callback
        and check final position is within RDBD.
        """

        try:
            caput(motor, position, wait=True, timeout=timeout)
        except:
            e = sys.exc_info()
            print str(e)
            print "ERROR: caput failed."
            print (motor + " pos:" + str(position) + " timeout:" + str(timeout))
            return self.__g.FAIL

        rdbd = motor + ".RDBD"
        rbv = motor + ".RBV"

        final_pos = caget(rbv)
        deadband = caget(rdbd)

        success = True

        if ((final_pos < position-deadband) or (final_pos > position+deadband)):
            print "ERROR: final_pos out of deadband."
            print (motor + " " + str(position) + " " + str(timeout) + " "
                   + str(final_pos) + " " + str(deadband))
            success = False

        if (success):
            return self.postMoveCheck(motor)
        else:
            self.postMoveCheck(motor)
            return self.__g.FAIL


    def movePosition(self, motor, tc_no, destination, velocity, acceleration):
        time_to_wait = 30
        if velocity > 0:
            distance = math.fabs(caget(motor + '.RBV') - destination)
            time_to_wait += distance / velocity + 2 * acceleration
        caput(motor + '.VAL', destination)
        done = self.waitForStartAndDone(motor, tc_no + " movePosition", time_to_wait)

    def setPosition(self, motor, position, timeout):
        """
        Set position on motor and check it worked ok.
        """

        _set = motor + ".SET"
        _rbv = motor + ".RBV"
        _dval = motor + ".DVAL"
        _off = motor + ".OFF"

        offset = caget(_off)

        caput(_set, 1, wait=True, timeout=timeout)
        caput(_dval, position, wait=True, timeout=timeout)
        caput(_off, offset, wait=True, timeout=timeout)
        caput(_set, 0, wait=True, timeout=timeout)

        if (self.postMoveCheck(motor) != self.__g.SUCCESS):
            return self.__g.FAIL

        try:
            self.verifyPosition(motor, position+offset)
        except Exception as e:
            print str(e)
            return self.__g.FAIL

    def checkInitRecord(self, motor):
        """
        Check the record for correct state at start of test.
        """
        self.postMoveCheck()


    def verifyPosition(self, motor, position):
        """
        Verify that field == reference.
        """
        _rdbd = motor + ".RDBD"
        deadband = caget(_rdbd)
        _rbv = motor + ".RBV"
        current_pos = caget(_rbv)

        if ((current_pos < position-deadband) or (current_pos > position+deadband)):
            print "ERROR: final_pos out of deadband."
            msg = (motor + " " + str(position) + " "
                   + str(current_pos) + " " + str(deadband))
            raise Exception(__name__ + msg)

        return self.__g.SUCCESS

    def verifyField(self, pv, field, reference):
        """
        Verify that field == reference.
        """
        full_pv = pv + "." + field
        if (caget(full_pv) != reference):
            msg = "ERROR: " + full_pv + " not equal to " + str(reference)
            raise Exception(__name__ + msg)

        return self.__g.SUCCESS


    def postMoveCheck(self, motor):
        """
        Check the motor for the correct state at the end of move.
        """

        DMOV = 1
        MOVN = 0
        STAT = 0
        SEVR = 0
        LVIO = 0
        MISS = 0
        RHLS = 0
        RLLS = 0

        try:
            self.verifyField(motor, "DMOV", DMOV)
            self.verifyField(motor, "MOVN", MOVN)
            self.verifyField(motor, "STAT", STAT)
            self.verifyField(motor, "SEVR", SEVR)
            self.verifyField(motor, "LVIO", LVIO)
            self.verifyField(motor, "MISS", MISS)
            self.verifyField(motor, "RHLS", RHLS)
            self.verifyField(motor, "RLLS", RLLS)
        except Exception as e:
            print str(e)
            return self.__g.FAIL

        return self.__g.SUCCESS




