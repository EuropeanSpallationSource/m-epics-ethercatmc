#
# Class to find an axis in the URL string
#
# The "axis" has a "connecton", that can communicate either
# - via EPICS channel access using pyepics
# - via pvacces access using p4p
# - via ADS using pyads
#


import datetime
import re
import sys

from AxisEPICS import AxisEPICS

filnam = "TestUtil"


def help_and_exit(self, url_string, problem_string):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Invalid URL ({url_string}) ({problem_string})"
    )
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Use e.g. ca://IOC:m1")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Use e.g. pva://IOC:m1")
    sys.exit(1)


class AxisTestUtil:
    def __init__(self, url_string, log_debug=False):
        self.conn = None
        if url_string.startswith("pva://") or url_string.startswith("ca://"):
            if url_string.startswith("ca://"):
                from ConnCA import ConnCA

                self.pvpfx = url_string[5:]
                self.conn = ConnCA(log_debug=log_debug)
            elif url_string.startswith("pva://"):
                from ConnP4P import ConnP4P

                self.pvpfx = url_string[6:]
                self.conn = ConnP4P(log_debug=log_debug)
            else:
                self.pvpfx = None

            self.url_string = url_string
            self.log_debug = log_debug
            import AxisEPICS as AxisEPICS
        else:
            help_and_exit(self, url_string, "invalid scheme")

    def close(self):
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} close() begin")
        self.conn.close()
        print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} close() done")

    def getAxis(self):
        axis = AxisEPICS(self.conn, self.pvpfx, log_debug=self.log_debug)
        return axis
