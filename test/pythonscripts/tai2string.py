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
    RE_MATCH_TAI = re.compile(
        #r"(\S*TAIclock\S*)\s+(\d+-\d*-\d*\s\d+:\d+:\d\.\d+)"
        r"(\S*TAI\S*)(\s+)(\d+-\d*-\d*\s+\d+:\d+:\d+\.\d+)(\s+)(\d+)$"
    )

    for line in sys.stdin:
        line = line.strip()
        match_tai = RE_MATCH_TAI.match(line)
        if match_tai != None:
            gidx = 1
            pvname = match_tai.group(gidx)
            gidx = gidx + 1
            sep1 = match_tai.group(gidx)
            gidx = gidx + 1
            datetime = match_tai.group(gidx)
            gidx = gidx + 1
            sep2 = match_tai.group(gidx)
            gidx = gidx + 1
            allnsec_plc_tai = int(match_tai.group(gidx))
            gidx = gidx + 1
            #print (f"{line} match_systemtaicloc")
            #print (f"pvname={pvname} datetime={datetime} allnsec_plc_tai={allnsec_plc_tai}")
            sec_tai_plc = int(allnsec_plc_tai / 1000000000)
            subnsec = allnsec_plc_tai % 1000000000
            alldatetime=time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(sec_tai_plc))
            epoch_now = int(time.time())
            #print(f"pvname={pvname} datetime={datetime} allnsec_plc_tai={allnsec_plc_tai} alldatetime={alldatetime}")
            #print(f"{pvname}{sep1}{datetime}{sep2}{alldatetime}.{subnsec:09d} {allnsec_plc_tai} {epoch_now} {sec_tai_plc}")
            print(f"{pvname}{sep1}{datetime}{sep2}{alldatetime}.{subnsec:09d} epoch-now={epoch_now} sec_tai_plc={sec_tai_plc}")
        else:
            print (line)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
