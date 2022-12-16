#
# Class to talk to one "axis", common code
#

import datetime
import inspect
import math
import re
import sys
import time

# import AxisEPICS
# from AxisEPICS import AxisEPICS

filnam = "AxisComm"


# def getComm(self, url_string, log_debug=False):
#    if url_string.startswith("pva://"):
#        axisComm = AxisEPICS(url_string, log_debug=log_debug)
#    elif url_string.startswith("ca://"):
#        axisComm = AxisEPICS(url_string, log_debug=log_debug)
#    else:
#        raise Exception("invalid url_string:" + url_string)
#    return axisComm


class AxisComm:
    # def __init__(self, url_string, axisNum=1, log_debug=True):
    #    super().__init__(url_string, axisNum=1, log_debug=True)
    #    #self.url_string = url_string
    def __init__(self, url_string):
        super().__init__(url_string)
        # self.url_string = url_string

    def calcAlmostEqual(self, expected, actual, maxdelta, tc_no=0, doPrint=False):
        fName = "calcAlmostEqual"

        delta = math.fabs(float(expected) - float(actual))
        delta <= maxdelta
        if delta <= maxdelta:
            inrange = True
        else:
            inrange = False
        if doPrint or not inrange:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} exp={expected} act={actual} delta={delta} maxdelta={maxdelta} inrange={inrange}"
            )
        return inrange

    def putDbgStrToLOG(self, value):
        raise Exception("putDbgStrToLOG not implemented")
