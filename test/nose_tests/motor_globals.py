#!/usr/bin/python

"""
Author: Matt Pearson
Date: Feb 2015

Description: Class to hold globals for motor example test scripts
"""

class motor_globals(object):
    """
    Class to hold globals for motor example test scripts.

    Constants are local variables to methods, and they are called
    by either the method name or the read-only property associated with that
    method.
    """

    def __init__(self):
        pass

    def getTimeout(self):
        return 100

    def getFail(self):
        return 1

    def getSuccess(self):
        return 0

    TIMEOUT = property(getTimeout, doc="Put callback timeout")
    FAIL = property(getFail, doc="Fail return value")
    SUCCESS = property(getSuccess, doc="Success return value")

