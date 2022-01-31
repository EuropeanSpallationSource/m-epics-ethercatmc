#!/usr/bin/env python3

import re
import sys

import time
import traceback
#from time import strftime
#from time import gmtime


#https://stackoverflow.com/questions/775049/how-do-i-convert-seconds-to-hours-minutes-and-seconds
#strftime("%H:%M:%S", gmtime(666))
#'00:11:06'



global debug
debug = False #True

########################################################################
#
# Record the name of the Aux Bits
#
class WrappingStepCounter:
    def __init__(self):
        self.old_steps = None
        self.wrap_around_offset = 0

    def set(self,value):
        steps = int(value)
        if debug:
            print(f"WrappingStepCounter steps={steps}")

        if self.old_steps != None:
            wrap_so_much = 65536
            if self.old_steps < -15000 and steps >= 15000:
                self.wrap_around_offset = self.wrap_around_offset - wrap_so_much
            elif self.old_steps > 15000 and steps <= -15000:
                self.wrap_around_offset = self.wrap_around_offset + wrap_so_much
        self.old_steps = steps
        self.unwrapped_steps = steps + self.wrap_around_offset
        if debug:
            print(f"WrappingStepCounter unwrapped_steps={self.unwrapped_steps}")

    def get_unwrapped(self):
        return self.unwrapped_steps

###################
RE_MATCH_IOC_PV_NOT_FOUND = re.compile(
    r".*( \*\*\* Not connected \(PV not found\))"
)
RE_MATCH_IOC_PV_DISCONNECTED = re.compile(r".*( \*\*\* disconnected| *<Disconnect>)")
RE_MATCH_INVALID_UNDEFINED = re.compile(r".*(INVALID|<undefined>).*")
# Regular expression to split the line into its components -
# Do a basic sanity test
#                                    name      date        time      raw
RE_MATCH_IOC_LOG_LINE = re.compile(r"(\S+)\s+([0-9-]+)\s+([0-9:.]+)\s+(.+)")

RE_MATCH_RAW_DIGITS_WITH_SIGN = re.compile(r"([0-9-]+)")
RE_MATCH_DIGIT_IN_PARANTHES = re.compile(r"\(([0-9-]+)\)")
#StatusCode 2021-12-14 16:47:39.780947 IDLE
RE_MATCH_ALPHA = re.compile(r"([A-Z]+)")

# LabS-MCAG:MC-MCU-07:m1-NamAuxBit0
RE_MATCH_NAMAUXBIT = re.compile(r"(.*)(NamAuxBit)([0-9]+)$")

#2021-12-16 15:08:01.163781 LabS-MCAG:MC-MCU-07:m1-StatusBits         9437185
RE_MATCH_STATUSBITS = re.compile(r".*-StatusBits(-TSE)?$")

#
RE_MATCH_RAWMTRSTEP = re.compile(r".*-RawMtrStep(-TSE)?$")

#
RE_MATCH_RAWMTRSTEP = re.compile(r".*-RawMtrStep(-TSE)?$")
#
RE_MATCH_RAWENCSTEP = re.compile(r".*-RawEncStep(-TSE)?$")

##############################################################
global auxbitnames
auxbitnames = {0:'AUXbit0',
               1:'AUXbit1',
               2:'AUXbit2',
               3:'AUXbit3',
               4:'AUXbit4',
               5:'AUXbit5',
               6:'AUXbit6',
               7:'AUXbit7',
               8:'AUXbit8',
               9:'AUXbit9',
               10:'AUXbit10',
               11:'AUXbit11',
               12:'AUXbit12',
               13:'AUXbit13',
               14:'AUXbit14',
               15:'AUXbit15',
               16:'AUXbit16',
               17:'AUXbit17',
               18:'AUXbit18',
               19:'AUXbit19',
               20:'AUXbit20',
               21:'AUXbit21',
               22:'AUXbit22',
               23:'AUXbit23',
               24:'AUXbit24',
               25:'AUXbit25',
               26:'AUXbit26'}


old_auxbits = 0

global wrappingMtrStepCounter
wrappingMtrStepCounter = WrappingStepCounter()
global wrappingEncStepCounter
wrappingEncStepCounter = WrappingStepCounter()



def format_line2(date, time, pvname, value):
    return f"{date} {time} {pvname:41} {value}"

########################################################################
#
# Record the name of the Aux Bits
#
def handle_namauxbit(line, match_namauxbit, raw):

    numauxbit = int(match_namauxbit.group(3))
    if debug:
        print(f"line={line} numauxbit={numauxbit} raw={raw}")
    auxbitnames[numauxbit] = raw
    return None

########################################################################
#
# the aux bits has changed
#
def handle_statusbits(date, time, pvname, raw):
    global old_auxbits
    new_auxbits = int(raw)
    bit_no = 24
    changed_txt = None


    #if debug or True:
    #    print(f"handle_statusbits")

    while bit_no > 0:
        bit_mask = 1 << bit_no
        if (new_auxbits ^ old_auxbits) & bit_mask:
            if new_auxbits & bit_mask:
                sign = '+'
            else:
                sign = '-'
            if changed_txt == None:
                changed_txt = sign + auxbitnames[bit_no]
            else:
                changed_txt = changed_txt + " " + sign + auxbitnames[bit_no]
        bit_no = bit_no -1

    if debug:
        print(f"handle_statusbits changed_txt={changed_txt}")
    value = f"0x{old_auxbits:07x}->0x{new_auxbits:07x} ({changed_txt})"
    line2 = format_line2(date, time, pvname, value)
    old_auxbits = new_auxbits
    return line2

########################################################################
#
# Motor raw steps
#
def handle_rawmtrstep(date, time, pvname, raw):

    if debug:
        print(f"handle_RawMtrStep raw={raw}")
    wrappingMtrStepCounter.set(raw)
    unwrapped_steps = wrappingMtrStepCounter.get_unwrapped()
    pvname2 = pvname + "-MT"
    return format_line2(date, time, pvname2, unwrapped_steps)

########################################################################
#
# Encoder raw steps
#
def handle_rawencstep(date, time, pvname, raw):

    if debug:
        print(f"handle_RawEncStep raw={raw}")
    wrappingEncStepCounter.set(raw)
    unwrapped_steps = wrappingEncStepCounter.get_unwrapped()
    pvname2 = pvname + "-MT"
    return format_line2(date, time, pvname2, unwrapped_steps)

########################################################################
#
# Handle ioc log line.
# Look at the special PVs, if any
#
def handle_ioc_log_line(line, pvname, date, time, raw):
    match_namauxbit = RE_MATCH_NAMAUXBIT.match(pvname)
    if match_namauxbit != None:
        return handle_namauxbit(line, match_namauxbit, raw)
    match_statusbits = RE_MATCH_STATUSBITS.match(pvname)
    if match_statusbits != None:
        return handle_statusbits(date, time, pvname, raw)
    match_rawmtrstep = RE_MATCH_RAWMTRSTEP.match(pvname)
    if match_rawmtrstep != None:
        return handle_rawmtrstep(date, time, pvname, raw)
    match_rawencstep = RE_MATCH_RAWENCSTEP.match(pvname)
    if match_rawencstep != None:
        return handle_rawencstep(date, time, pvname, raw)

    # Nothing special: Return the line. data/time first
    return format_line2(date, time, pvname, raw)

########################################################################
#
#  Handle any line: Filter out "disconnected" and out stuff
#
def handle_any_line(line):
    match_ioc_pv_not_found = RE_MATCH_IOC_PV_NOT_FOUND.match(line)
    if match_ioc_pv_not_found != None:
        return None
    match_ioc_pv_disconnected = RE_MATCH_IOC_PV_DISCONNECTED.match(line)
    if match_ioc_pv_disconnected != None:
        return None
    match_invalid_undefined = RE_MATCH_INVALID_UNDEFINED.match(line)
    if match_invalid_undefined != None:
        return None

    match_ioc_log_line = RE_MATCH_IOC_LOG_LINE.match(line)
    if match_ioc_log_line == None:
        print("re_match == None line=%s" % (line))
        sys.exit(1)

    pvname = match_ioc_log_line.group(1)
    date = match_ioc_log_line.group(2)
    time = match_ioc_log_line.group(3)
    raw = match_ioc_log_line.group(4)
    if debug:
        print(
            "match_ioc_log_line=%s pvname=%s date=%s time=%s raw='%s' "
            % (match_ioc_log_line, pvname, date, time, raw)
        )
    return handle_ioc_log_line(line, pvname, date, time, raw)


def main(argv=None):
    if not argv:
        argv = sys.argv


    for line in sys.stdin:
        line = line.strip()
        try:
            line2 = handle_any_line(line)
            if line2 != None:
                print (line2)

        except Exception as e:
            print("line=%s" % (line))
            print("line.split=%s" % (line.split(" ")))
            print(str(e))
            #traceback.print_exception(e, None)
            traceback.print_exc()
            sys.exit(1)

        sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
