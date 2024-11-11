#
# Class to talk to one "axis"
#
# The "axis" has a "connecton", that can communicate either
# - via EPICS channel access using pyepics
# - via pvacces access using p4p
#

import datetime
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
    def __init__(self, url_string, axisNum=1, log_debug=True, monitor_list=[".DMOV"]):
        self.log_debug = log_debug
        self.monitor_list = monitor_list
        self.pvpfx = None  # PV prefix, like IOC:m1
        self.ctxt = None  # P4P context, if any
        self.pv_dmov_ca = None  # needed for monitor in ca/pyepics
        self.sub = None  # needed for monitor in pva/p4p

        self.change_cnts = {}  # count number of callbacks
        self.init_change_cnts()

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
        # set up monitor, if needed
        if self.monitor_list is not None:
            if monitor_list.count(".DMOV"):
                if self.ctxt is not None:
                    # pva/p4p
                    self.sub = self.ctxt.monitor(
                        self.pvpfx + ".DMOV", self.onChangesDmovPVA
                    )
                else:
                    # ca/pyepics
                    from epics import PV

                    self.pv_dmov_ca = PV(self.pvpfx + ".DMOV")
                    self.pv_dmov_ca.add_callback(callback=self.onChangesCA)

    def close(self):
        if self.ctxt is not None:
            if self.sub is not None:
                self.sub.close()
            self.ctxt.close()
            self.ctxt = None
        if self.pv_dmov_ca is not None:
            self.pv_dmov_ca.clear_callbacks()

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
        if as_string:
            raise Exception("as_string=True not supported")
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

    def get_change_cnts(self, counterName):
        return self.change_cnts[counterName]

    def getMotorPvName(self):
        return self.pvpfx

    def init_change_cnts(self):
        self.change_cnts["dmov_value"] = 1  # the .DMOV field is true
        self.change_cnts["dmov_false"] = 0  # the .DMOV field went false
        self.change_cnts["dmov_true"] = 0  # the .DMOV field went true

    def onChangesCA(self, pvname=None, value=None, char_value=None, **kwx):
        old_val = int(self.change_cnts["dmov_value"])
        if value != old_val:
            if int(value) == 0:
                self.change_cnts["dmov_false"] += 1
            elif int(value) == 1:
                self.change_cnts["dmov_true"] += 1
            self.change_cnts["dmov_value"] = int(value)
        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} onChangesCA: pvname={pvname} value={value} old_val={old_val} dmov_false={self.change_cnts['dmov_false']} dmov_true={self.change_cnts['dmov_true']}"
            )

    def onChangesDmovPVA(self, value):
        old_val = int(self.change_cnts["dmov_value"])
        if value != old_val:
            if int(value) == 0:
                self.change_cnts["dmov_false"] += 1
            elif int(value) == 1:
                self.change_cnts["dmov_true"] += 1
            self.change_cnts["dmov_value"] = int(value)

        if self.log_debug:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} onChangesPVA: value={int(value)} old_val={old_val} dmov_false={self.change_cnts['dmov_false']} dmov_true={self.change_cnts['dmov_true']}"
            )

    def put(self, pvsuf, value, wait=False, timeout=5.0, throw=True):
        pvname = self.pvpfx + pvsuf
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
        if self.ctxt is not None:
            # p4p
            ret = self.ctxt.put(pvname, value, timeout=timeout, wait=wait, throw=throw)
            if self.log_debug:
                if ret is None:
                    print(
                        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} value={value}"
                    )
                else:
                    print(
                        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} value={value} ret={ret}"
                    )
            if ret is None:
                return True
            return False

        else:
            # pyepics
            caput_ret = self.epics.caput(pvname, value, timeout=timeout, wait=wait)
            # This function returns 1 on success,
            # and a negative number if the timeout has been exceeded
            if self.log_debug:
                print(
                    f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {fullname} value={value} caput_ret={ret}"
                )
            if caput_ret != 1:
                if throw:
                    raise Exception(
                        f"caput({pvname},{value}) returned error {caput_ret}"
                    )
                else:
                    return False
            return True

    def putDbgStrToLOG(self, value, wait=True, timeout=5.0):
        pvsuf = "-DbgStrToLOG"
        try:
            self.put(pvsuf, value, wait=wait, timeout=timeout)
        except Exception as ex:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} put {pvsuf} value={value} ex={ex}"
            )
