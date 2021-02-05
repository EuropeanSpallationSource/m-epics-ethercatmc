#!/usr/bin/env python3

import re
import sys

import time
#from time import strftime
#from time import gmtime

#https://stackoverflow.com/questions/775049/how-do-i-convert-seconds-to-hours-minutes-and-seconds
#strftime("%H:%M:%S", gmtime(666))
#'00:11:06'


def main(argv=None):
    global debug
    if not argv:
        argv = sys.argv

#LabS-MCAG:MC-MCU-07:nSystemTAIclock 2021-02-03 07:52:39.632  1612338796622117100
    RE_MATCH_SYSTEMTAICLOCK = re.compile(
        #r"(\S*TAIclock\S*)\s+(\d+-\d*-\d*\s\d+:\d+:\d\.\d+)"
        r"(\S*TAI\S*)(\s+)(\d+-\d*-\d*\s+\d+:\d+:\d+\.\d+)(\s+)(\d+)$"
    )

    for line in sys.stdin:
        line = line.strip()
        match_systemtaiclock = RE_MATCH_SYSTEMTAICLOCK.match(line)
        if match_systemtaiclock != None:
            gidx = 1
            pvname = match_systemtaiclock.group(gidx)
            gidx = gidx + 1
            sep1 = match_systemtaiclock.group(gidx)
            gidx = gidx + 1
            datetime = match_systemtaiclock.group(gidx)
            gidx = gidx + 1
            sep2 = match_systemtaiclock.group(gidx)
            gidx = gidx + 1
            allnanoseconds = int(match_systemtaiclock.group(gidx))
            gidx = gidx + 1
            #print (f"{line} match_systemtaicloc")
            #print (f"pvname={pvname} datetime={datetime} allnanoseconds={allnanoseconds}")
            seconds = int(allnanoseconds / 1000000000)
            subnsec = allnanoseconds % 1000000000
            alldatetime=time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(seconds))
            epoch_or_so = int(time.time())
            #print(f"pvname={pvname} datetime={datetime} allnanoseconds={allnanoseconds} alldatetime={alldatetime}")
            #print(f"{pvname}{sep1}{datetime}{sep2}{alldatetime}.{subnsec:09d} {allnanoseconds} {epoch_or_so} {seconds}")
            print(f"{pvname}{sep1}{datetime}{sep2}{alldatetime}.{subnsec:09d} epoch={epoch_or_so} tai={seconds}")
        else:
            print (line)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
