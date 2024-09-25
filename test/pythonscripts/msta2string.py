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


def getMSTAtext(msta):
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


def main(argv=None):
    global debug
    if not argv:
        argv = sys.argv

    # 2024/09/24 17:16:30.491 [motorRecord.cc:1787 LabS-MCAG:MC-MCU-10:m1 01] motor is stopped dval=0.000000 drbv=0.330000 pp=1 udf=0 stat=7 stop=0 pmr->spmg=GO mip=0x8(HOMF(Hf)) msta=0xb05
    RE_MATCH_MSTA = re.compile(r"(.*\s)(msta=0x)([0-9a-fA-F]*)(.*)$")

    for line in sys.stdin:
        line = line.strip()
        match_msta = RE_MATCH_MSTA.match(line)
        if match_msta is not None:
            gidx = 1
            loginfo1 = match_msta.group(gidx)
            gidx = gidx + 1
            msta_equals = match_msta.group(gidx)
            gidx = gidx + 1
            msta_val_hex = match_msta.group(gidx)
            gidx = gidx + 1
            loginfo2 = match_msta.group(gidx)
            msta_val_int = int(msta_val_hex, 16)
            msta_val_str = getMSTAtext(msta_val_int)
            print(f"{loginfo1}{msta_equals}{msta_val_hex} ({msta_val_str}) {loginfo2}")
        else:
            print(line)
        sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
