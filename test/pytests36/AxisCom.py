#
# Class to talk to one "axis"
#
# The "axis" has a "connecton", that can communicate either
# - via EPICS channel access using pyepics
# - via pvacces access using p4p
# - via ADS using pyads
#

import datetime
import re
import sys

filnam = "AxisCom"


def help_and_exit(self, url_string, problem_string):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Invalid URL ({url_string}) ({problem_string})"
    )
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Use e.g. ca://IOC:m1")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Use e.g. pva://IOC:m1")
    sys.exit(1)


class AxisCom:
    def __init__(self, url_string, axisNum=1, log_debug=True):
        self.pvpfx = None  # PV prefix, like IOC:m1
        self.ctxt = None  # P4P context, if any
        self.log_debug = log_debug
        if url_string.startswith("pva://"):
            self.url_scheme = "pva://"
            self.pvpfx = url_string[6:]
            from p4p.client.thread import Context

            self.ctxt = Context("pva")
        elif url_string.startswith("ca://"):
            # Channel access
            self.url_scheme = "ca://"
            self.pvpfx = url_string[5:]
            import epics as epics

            self.epics = epics
        else:
            help_and_exit(self, url_string, "invalid scheme")

    def get(
        self,
        pvsuf,
        as_string=False,
        count=None,
        as_numpy=True,
        timeout=25.0,
        use_monitor=False,
    ):
        pvname = self.pvpfx + pvsuf
        fullname = self.url_scheme + pvname
        ret = None
        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} get {fullname}"
            )
        if self.ctxt is not None:
            ret = self.ctxt.get(pvname, timeout=timeout)
        else:
            ret = self.epics.caget(pvname, timeout=timeout, use_monitor=use_monitor)

        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} get {fullname} ret={ret}"
            )

        if ret is None:
            raise Exception("get None")
        return ret

    def put(
        self,
        pvsuf,
        value,
        wait=False,
        timeout=5.0,
    ):
        pvname = self.pvpfx + pvsuf
        fullname = self.url_scheme + pvname
        ret = None
        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} value={value}"
            )
        if self.ctxt is not None:
            self.ctxt.put(pvname, value, timeout=timeout, wait=wait)
        else:
            caput_ret = self.epics.caput(pvname, value, timeout=timeout, wait=wait)
            # This function returns 1 on success,
            # and a negative number if the timeout has been exceeded
            if caput_ret != 1:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} value={value} caput_ret={ret}"
                )
                raise Exception(f"caput({pvname},{value}) returned error {caput_ret}")
