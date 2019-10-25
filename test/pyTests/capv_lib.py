import epics
#import pvapy
#from pvapy import pvaccess


#
ctxt = None
##

# Wrapper arounf epics caget() and caput()
# That it is why we call it pvXXX()
# However, by having a wrapper we can switch to pcAccess later
def capvget(pvname, as_string=False, count=None, as_numpy=True, timeout=5.0, use_monitor=False):
    global ctxt
    #return epics.caget(pvname, as_string=as_string, count=count, as_numpy=as_numpy, timeout=timeout, use_monitor=use_monitor)
    #pvaccess.Channel(PVNAME, pvaccess.CA).get().getPyObject()

    #channel = pvaccess.Channel(pvname, pvaccess.CA)
    #return channel.get().getPyObject()
    if pvname.startswith('pva://'):
        if ctxt is None:
            from p4p.client.thread import Context
            ctxt = Context('pva')
        return ctxt.get(pvname[6:])
    else:
        return epics.caget(pvname)

def capvput(pvname, value, wait=False, timeout=5.0):
    global ctxt
    #channel = pvaccess.Channel(pvname, pvaccess.CA)
    #return channel.putFloat(value)
    if pvname.startswith('pva://'):
        if ctxt is None:
            from p4p.client.thread import Context
            ctxt = Context('pva')
        ctxt.put(pvname[6:], value, timeout=timeout, wait=wait)
    else:
        epics.caput(pvname, value, timeout=timeout, wait=wait)

