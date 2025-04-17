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
MSTA_BIT_ENC_HOME = 1 << (8 - 1)  # 0080
MSTA_BIT_SLIP_STALL = 1 << (7 - 1)  # 0040
MSTA_BIT_AMPON = 1 << (6 - 1)  # 0020
MSTA_BIT_UNUSED = 1 << (5 - 1)  # 0010
MSTA_BIT_HOME_SWITCH = 1 << (4 - 1)  # 0008
MSTA_BIT_PLUS_LS = 1 << (3 - 1)  # 0004
MSTA_BIT_DONE = 1 << (2 - 1)  # 0002
MSTA_BIT_DIRECTION = 1 << (1 - 1)  # 0001


# MIP_DONE        0x0000  /* No motion is in progress. */
MIP_JOGF = 1 << (1 - 1)  # 0001
MIP_JOGR = 1 << (2 - 1)  # 0001
MIP_JOG_BL1 = 1 << (3 - 2)  # 0002
MIP_HOMF = 1 << (4 - 1)  # 0004
MIP_HOMR = 1 << (5 - 1)  # 0008
MIP_MOVE = 1 << (6 - 1)  # 0010
MIP_RETRY = 1 << (7 - 1)  # 0020
MIP_LOAD_P = 1 << (8 - 1)  # 0040
MIP_MOVE_BL = 1 << (9 - 1)  # 0080
MIP_STOP = 1 << (10 - 1)  # 0100
MIP_DELAY_ACK = 1 << (11 - 1)  # 0200
MIP_JOG_REQ = 1 << (12 - 1)  # 0400
MIP_JOG_STOP = 1 << (13 - 1)  # 0800
MIP_JOG_BL2 = 1 << (14 - 1)  # 1000
MIP_EXTERNAL = 1 << (15 - 1)  # 2000


def get_mip_text_actual(mip):
    if mip == 0:
        return "Done"
    ret = ""
    if mip & MIP_JOGF:
        ret = ret + "Jogf"
    if mip & MIP_JOGR:
        ret = ret + "Jogr"
    if mip & MIP_JOG_BL1:
        ret = ret + "JogBL1"
    if mip & MIP_HOMF:
        ret = ret + "Homf"
    if mip & MIP_HOMR:
        ret = ret + "Homr"
    if mip & MIP_MOVE:
        ret = ret + "Move"
    if mip & MIP_RETRY:
        ret = ret + "Retry"
    if mip & MIP_LOAD_P:
        ret = ret + "Loadp"
    if mip & MIP_MOVE_BL:
        ret = ret + "MovBl"
    if mip & MIP_STOP:
        ret = ret + "Stop"
    if mip & MIP_DELAY_ACK:
        ret = ret + "Delayack"
    if mip & MIP_JOG_REQ:
        ret = ret + "Jogreq"
    if mip & MIP_JOG_STOP:
        ret = ret + "Stop"
    if mip & MIP_JOG_BL2:
        ret = ret + "BL2"
    if mip & MIP_EXTERNAL:
        ret = ret + "External"
    return ret


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
    if msta & MSTA_BIT_COMM_ERR:
        ret = ret + "Com"
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
    if msta & MSTA_BIT_ENC_HOME:
        ret = ret + "EHm"
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
    if msta & MSTA_BIT_HOME_SWITCH:
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
            ret = ret + "+Homed"
        else:
            ret = ret + "-Homed"
    if (old_msta ^ new_msta) & MSTA_BIT_MINUS_LS:
        if new_msta & MSTA_BIT_MINUS_LS:
            ret = ret + "+LLS"
        else:
            ret = ret + "-LLS"
    if (old_msta ^ new_msta) & MSTA_BIT_COMM_ERR:
        if new_msta & MSTA_BIT_COMM_ERR:
            ret = ret + "+ComError"
        else:
            ret = ret + "-ComError"
    if (old_msta ^ new_msta) & MSTA_BIT_GAIN_SUPPORT:
        if new_msta & MSTA_BIT_GAIN_SUPPORT:
            ret = ret + "+GainSupport"
        else:
            ret = ret + "-GainSupport"
    if (old_msta ^ new_msta) & MSTA_BIT_MOVING:
        if new_msta & MSTA_BIT_MOVING:
            ret = ret + "+Moving"
        else:
            ret = ret + "-Moving"
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
    if (old_msta ^ new_msta) & MSTA_BIT_ENC_HOME:
        if new_msta & MSTA_BIT_ENC_HOME:
            ret = ret + "+EncHome"
        else:
            ret = ret + "-EncHome"
    if (old_msta ^ new_msta) & MSTA_BIT_SLIP_STALL:
        if new_msta & MSTA_BIT_SLIP_STALL:
            ret = ret + "+SlipStall"
        else:
            ret = ret + "-SlipStall"
    if (old_msta ^ new_msta) & MSTA_BIT_AMPON:
        if new_msta & MSTA_BIT_AMPON:
            ret = ret + "+Poweron"
        else:
            ret = ret + "-Poweron"
    if (old_msta ^ new_msta) & MSTA_BIT_HOME_SWITCH:
        if new_msta & MSTA_BIT_HOME_SWITCH:
            ret = ret + "+HomeSwitch"
        else:
            ret = ret + "-HomeSwitch"
    if (old_msta ^ new_msta) & MSTA_BIT_PLUS_LS:
        if new_msta & MSTA_BIT_PLUS_LS:
            ret = ret + "+HLS"
        else:
            ret = ret + "-HLS"
    if (old_msta ^ new_msta) & MSTA_BIT_DONE:
        if new_msta & MSTA_BIT_DONE:
            ret = ret + "+Done"
        else:
            ret = ret + "-Done"
    if (old_msta ^ new_msta) & MSTA_BIT_DIRECTION:
        if new_msta & MSTA_BIT_DIRECTION:
            ret = ret + "+Direction"
        else:
            ret = ret + "-Direction"

    if old_msta == 0:
        ret = ret.replace("+", " ")
        ret = ret.strip()
    return ret


# 2024/09/24 17:16:30.491 [motorRecord.cc:1787 LabS-MCAG:MC-MCU-10:m1 01] motor is stopped dval=0.000000 drbv=0.330000 pp=1 udf=0 stat=7 stop=0 pmr->spmg=GO mip=0x8(HOMF(Hf)) msta=0xb05
RE_MATCH_MSTA_IOCLOG_MR = re.compile(
    r"(.*motorRecord.cc:\d+\s+)(\S+)(.*)(msta=0x)([0-9a-fA-F]*)(.*)$"
)

RE_MATCH_MOTOR_IS_STOPPED = re.compile(r".*motor is stopped")

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

# SQ:SINQTEST:mmacs1:lin1.MIP    2025-02-04 18:18:19.694284 0
RE_MATCH_MIP_CAMONITOR_DATE = re.compile(
    "(\S+\.MIP)\s+([-0-9]+\s+[0-9.:]+)\s+([0-9]+)\s*(.*)$"
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
    match_motor_is_stopped = RE_MATCH_MOTOR_IS_STOPPED.match(loginfo2)
    if match_motor_is_stopped is not None:
        last_msta_int = int(0)
        pfx = "s:"
    elif last_msta_int is None:
        last_msta_int = int(0)
        pfx = "o:"
    else:
        pfx = "m:"
    msta_delta = get_msta_text_delta(last_msta_int, msta_val_int)
    print(
        f"{pfx}{loginfo1}{pvname}{loginfo2}{msta_equals}{msta_val_hex}({msta_delta}){loginfo3}"
    )
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


def handle_camonitor_msta_line(match_msta_camonitor):
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


def handle_camonitor_mip_line(match_mip_camonitor):
    gidx = 1
    pvname = match_mip_camonitor.group(gidx)
    gidx = gidx + 1
    time_date = match_mip_camonitor.group(gidx)
    gidx = gidx + 1
    mip_val_int = int(match_mip_camonitor.group(gidx))
    gidx = gidx + 1
    rest = match_mip_camonitor.group(gidx)
    mip_str = get_mip_text_actual(mip_val_int)
    print(f"M:{pvname:30s} {time_date} 0x{mip_val_int:04x} ({mip_str}){rest}")
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
            handle_camonitor_msta_line(match_msta_camonitor)
            continue
        match_mip_camonitor = RE_MATCH_MIP_CAMONITOR_DATE.match(line)
        if match_mip_camonitor is not None:
            handle_camonitor_mip_line(match_mip_camonitor)
            continue
        print(f"L:{line}")
        sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
