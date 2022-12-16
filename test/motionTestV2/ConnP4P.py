import datetime
import sys
import time

from ConnEPICS import ConnEPICS
from p4p.client.thread import Context

filnam = "ConnP4P"


class ConnP4P(ConnEPICS):
    def __init__(self, log_debug=False):
        self.ctxt = Context("pva")
        self.log_debug = log_debug
        self.url_scheme = "pva://"

    def close(self):
        self.ctxt.close
        self.ctxt = None

    def get(
        self,
        pvname,
        as_string=False,
        count=None,
        as_numpy=True,
        timeout=25.0,
        use_monitor=False,
    ):
        ret = None
        start = time.time()
        if as_string == True:
            raise Exception("as_string=True not supported")
        if self.log_debug:
            print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} get {pvname}")
        ret = self.ctxt.get(pvname, timeout=timeout)
        if self.log_debug:
            delta = time.time() - start
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} get {pvname} ret={ret} time={delta:.2f}"
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
        ret = None
        start = time.time()
        if self.log_debug:
            if wait:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {pvname} timeout={timeout} wait={wait} value={value}"
                )
            else:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {pvname} value={value}"
                )
        ret = self.ctxt.put(pvname, value, timeout=timeout, wait=wait)
        if self.log_debug:
            delta = time.time() - start
            if ret == None:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {pvname} value={value} time={delta:.2f}"
                )
            else:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {pvname} value={value} ret={ret} time={delta:.2f}"
                )
