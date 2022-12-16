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

polltime = 0.2
filnam = "ConnEPICS"


def lineno():
    return inspect.currentframe().f_back.f_lineno


class ConnEPICS:
    def __init__(self, url_string, log_debug=False):
        self.url_string = url_string
        self.log_debug = log_debug

    def get(
        self,
        pvname,
        as_string=False,
        count=None,
        as_numpy=True,
        timeout=25.0,
        use_monitor=False,
    ):
        raise NotImplementedError("Subclass has not overwritten method {method}!")

    def put(
        self,
        pvname,
        value,
        wait=False,
        timeout=5.0,
    ):
        raise NotImplementedError("Subclass has not overwritten method {method}!")
