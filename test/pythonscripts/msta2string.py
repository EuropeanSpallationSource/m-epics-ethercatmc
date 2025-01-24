#!/usr/bin/env python3


# script to translate msta=0xb05 into a readable format
import re
import sys

MSTA_BIT_HOMED = 1 << (15 - 1)  # 4000
MSTA_BIT_MINUS_LS = 1 << (14 - 1)  # 2000
MSTA_BIT_COMM_ERR = 1 << (13 - 1)  # 1000
MSTA_BIT_GAIN_SUPPORT = 1 << (12 - 1)  # 0800
MSTA_BIT_MOVING = 1 << (11 - 1)  # 0400
MSTA_BIT_PROBLEM = 1 << (10 - 1)  # 0200
MSTA_BIT_PRESENT = 1 << (9 - 1)  # 0100
MSTA_BIT_HOME = 1 << (8 - 1)  # 0080
MSTA_BIT_SLIP_STALL = 1 << (7 - 1)  # 0040
MSTA_BIT_AMPON = 1 << (6 - 1)  # 0020
MSTA_BIT_UNUSED = 1 << (5 - 1)  # 0010
MSTA_BIT_HOMELS = 1 << (4 - 1)  # 0008
MSTA_BIT_PLUS_LS = 1 << (3 - 1)  # 0004
MSTA_BIT_DONE = 1 << (2 - 1)  # 0002
MSTA_BIT_DIRECTION = 1 << (1 - 1)  # 0001


def get_msta_text_actual(msta):
    ret = ""
    if msta & MSTA_BIT_HOMED:
        ret = ret + "Hmd"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_MINUS_LS:
        ret = ret + "Lls"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_GAIN_SUPPORT:
        ret = ret + "Gain"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_MOVING:
        ret = ret + "Mov"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_PROBLEM:
        ret = ret + "Prb"
    else:
        ret = ret + "."
    if msta & MSTA_BIT_PRESENT:
        ret = ret + "Enc"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_HOME:
        ret = ret + "Hom"
    else:
        ret = ret + ".."
    if msta & MSTA_BIT_SLIP_STALL:
        ret = ret + "Slp"
    else:
        ret = ret + "...."
    if msta & MSTA_BIT_AMPON:
        ret = ret + "Amp"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_HOMELS:
        ret = ret + "Hsw"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_PLUS_LS:
        ret = ret + "Hls"
    else:
        ret = ret + "..."
    if msta & MSTA_BIT_DONE:
        ret = ret + "Don"
    else:
        ret = ret + "..."
    return ret


def get_msta_text_delta(old_msta, new_msta):
    ret = ""
    if (old_msta ^ new_msta) & MSTA_BIT_HOMED:
        if new_msta & MSTA_BIT_HOMED:
            ret = ret + "+Hmd"
        else:
            ret = ret + "-Hmd"
    if (old_msta ^ new_msta) & MSTA_BIT_MINUS_LS:
        ret = ret + "xLls"
    if (old_msta ^ new_msta) & MSTA_BIT_GAIN_SUPPORT:
        if new_msta & MSTA_BIT_GAIN_SUPPORT:
            ret = ret + "+GainSupport"
        else:
            ret = ret + "-GainSupport"
    if (old_msta ^ new_msta) & MSTA_BIT_MOVING:
        ret = ret + "xMoving"
    if (old_msta ^ new_msta) & MSTA_BIT_PROBLEM:
        if new_msta & MSTA_BIT_PROBLEM:
            ret = ret + "+Problem"
        else:
            ret = ret + "-Problem"
    if (old_msta ^ new_msta) & MSTA_BIT_PRESENT:
        if new_msta & MSTA_BIT_PRESENT:
            ret = ret + "+Encoder"
        else:
            ret = ret + "-Encoder"
    if (old_msta ^ new_msta) & MSTA_BIT_HOME:
        ret = ret + "xHome"
    if (old_msta ^ new_msta) & MSTA_BIT_SLIP_STALL:
        ret = ret + "xSlip"
    if (old_msta ^ new_msta) & MSTA_BIT_AMPON:
        if new_msta & MSTA_BIT_AMPON:
            ret = ret + "+Poweron"
        else:
            ret = ret + "-Poweron"
    if (old_msta ^ new_msta) & MSTA_BIT_HOMELS:
        ret = ret + "xHomeSwitch"
    if (old_msta ^ new_msta) & MSTA_BIT_PLUS_LS:
        ret = ret + "xHls"
    if (old_msta ^ new_msta) & MSTA_BIT_DONE:
        if new_msta & MSTA_BIT_DONE:
            ret = ret + "+Done"
        else:
            ret = ret + "-Done"

    return ret


# 2024/09/24 17:16:30.491 [motorRecord.cc:1787 LabS-MCAG:MC-MCU-10:m1 01] motor is stopped dval=0.000000 drbv=0.330000 pp=1 udf=0 stat=7 stop=0 pmr->spmg=GO mip=0x8(HOMF(Hf)) msta=0xb05
RE_MATCH_MSTA_IOCLOG_MR = re.compile(
    r"(.*motorRecord.cc:\d+\s+)(\S+)(.*)(msta=0x)([0-9a-fA-F]*)(.*)$"
)

RE_MATCH_MSTA_IOCLOG_MSTA = re.compile(r"(.*\s)(msta=0x)([0-9a-fA-F]*)(.*)$")

# xxx:hxp:c0:m3.MSTA             2025-01-20 17:57:02.486434 18722
# xxx:hxp:c0:m3.MSTA             2025-01-20 18:06:46.609890 2818 STATE MAJOR
RE_MATCH_MSTA_CAMONITOR_DATE = re.compile(
    "(\S+\.MSTA)\s+([-0-9]+\s+[0-9.:]+)\s+([0-9]+)\s*(.*)$"
)
# xxx:hxp:c0:m3.MSTA             <undefined> 0 UDF INVALID
RE_MATCH_MSTA_CAMONITOR_UNDEFINED = re.compile(
    "(\S+\.MSTA)\s+([^0-9:. ]+)\s+([0-9]+)\s*(.*)$"
)

last_msta_per_pv = {}  # Save the (last) value in order to calc a diff


########################################################################
def handle_ioclog_mr_line(match_msta_ioclog_mr):
    gidx = 1
    loginfo1 = match_msta_ioclog_mr.group(gidx)
    gidx = gidx + 1
    pvname = match_msta_ioclog_mr.group(gidx)
    gidx = gidx + 1
    loginfo2 = match_msta_ioclog_mr.group(gidx)
    gidx = gidx + 1
    msta_equals = match_msta_ioclog_mr.group(gidx)
    gidx = gidx + 1
    msta_val_hex = match_msta_ioclog_mr.group(gidx)
    gidx = gidx + 1
    loginfo3 = match_msta_ioclog_mr.group(gidx)
    msta_val_int = int(msta_val_hex, 16)
    last_msta_int = last_msta_per_pv.get(pvname)
    if last_msta_int is None:
        last_msta_int = int(0)
        pfx = "o:"
    else:
        pfx = "m:"
    if last_msta_int != msta_val_int:
        msta_delta = get_msta_text_delta(last_msta_int, msta_val_int)
        print(
            f"{pfx}{loginfo1}{pvname}{loginfo2}{msta_equals}{msta_val_hex}({msta_delta}){loginfo3}"
        )
    else:
        print(f"{pfx}{loginfo1}{pvname}{loginfo2}{loginfo3}")
    last_msta_per_pv[pvname] = msta_val_int
    sys.stdout.flush()


def handle_ioclog_msta_line(match_msta_ioclog_msta):
    gidx = 1
    loginfo1 = match_msta_ioclog_msta.group(gidx)
    gidx = gidx + 1
    msta_equals = match_msta_ioclog_msta.group(gidx)
    gidx = gidx + 1
    msta_val_hex = match_msta_ioclog_msta.group(gidx)
    gidx = gidx + 1
    loginfo2 = match_msta_ioclog_msta.group(gidx)
    msta_val_int = int(msta_val_hex, 16)
    msta_val_str = get_msta_text_actual(msta_val_int)
    print(f"I:{loginfo1}{msta_equals}{msta_val_hex} ({msta_val_str}) {loginfo2}")
    sys.stdout.flush()


def handle_camonitor_line(match_msta_camonitor):
    gidx = 1
    pvname = match_msta_camonitor.group(gidx)
    gidx = gidx + 1
    time_date = match_msta_camonitor.group(gidx)
    gidx = gidx + 1
    msta_val_int = int(match_msta_camonitor.group(gidx))
    gidx = gidx + 1
    rest = match_msta_camonitor.group(gidx)
    last_msta_int = last_msta_per_pv.get(pvname)
    if last_msta_int is not None:
        msta_delta = get_msta_text_delta(last_msta_int, msta_val_int)
        print(f"N:{pvname:30s} {time_date} 0x{msta_val_int:04x} ({msta_delta}){rest}")
    else:
        last_msta_int = int(0)
        msta_delta = get_msta_text_delta(last_msta_int, msta_val_int)
        print(f"O:{pvname:30s} {time_date} 0x{msta_val_int:04x} ({msta_delta}){rest}")
    last_msta_per_pv[pvname] = msta_val_int
    sys.stdout.flush()


def main(argv=None):
    global debug
    if not argv:
        argv = sys.argv

    for line in sys.stdin:
        line = line.strip()
        # check for ioc logs from motorRecord first
        match_msta_ioclog_mr = RE_MATCH_MSTA_IOCLOG_MR.match(line)
        if match_msta_ioclog_mr is not None:
            handle_ioclog_mr_line(match_msta_ioclog_mr)
            continue
        match_msta_ioclog_msta = RE_MATCH_MSTA_IOCLOG_MSTA.match(line)
        if match_msta_ioclog_msta is not None:
            handle_ioclog_msta_line(match_msta_ioclog_msta)
            continue
        match_msta_camonitor = RE_MATCH_MSTA_CAMONITOR_DATE.match(line)
        if match_msta_camonitor is None:
            match_msta_camonitor = RE_MATCH_MSTA_CAMONITOR_UNDEFINED.match(line)
        if match_msta_camonitor is not None:
            handle_camonitor_line(match_msta_camonitor)
            continue
        print(f"L:{line}")
        sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
