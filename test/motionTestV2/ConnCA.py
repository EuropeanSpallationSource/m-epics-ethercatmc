import datetime
import sys

import epics as epics
from ConnEPICS import ConnEPICS

filnam = "ConnCA"


def help_and_exit(self, url_string, problem_string):
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Invalid URL ({url_string}) ({problem_string})"
    )
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} Use e.g. ca://IOC:m1")
    sys.exit(1)


class ConnCA(ConnEPICS):
    def __init__(self, log_debug=False):
        self.log_debug = log_debug
        self.url_scheme = "ca://"

    def close(self):
        self.url_scheme = None  # Needs better code

    def get(
        self,
        pvname,
        as_string=False,
        count=None,
        as_numpy=True,
        timeout=25.0,
        use_monitor=False,
    ):
        fullname = self.url_scheme + pvname
        ret = None
        if as_string == True:
            raise Exception("as_string=True not supported")
        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} get {fullname}"
            )
        ret = epics.caget(pvname, timeout=timeout, use_monitor=use_monitor)
        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} get {fullname} ret={ret}"
            )

        if ret is None:
            raise Exception("get None")
        return ret

    def put(
        self,
        pvname,
        value,
        wait=False,
        timeout=5.0,
    ):
        fullname = self.url_scheme + pvname
        ret = None
        if self.log_debug:
            if wait:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} timeout={timeout} wait={wait} value={value}"
                )
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} value={value}"
            )
        caput_ret = epics.caput(pvname, value, timeout=timeout, wait=wait)
        # This function returns 1 on success,
        # and a negative number if the timeout has been exceeded
        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} value={value} caput_ret={ret}"
            )
        if caput_ret != 1:
            raise Exception(f"caput({pvname},{value}) returned error {caput_ret}")
