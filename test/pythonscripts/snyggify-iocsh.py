#!/usr/bin/env python3

#
# Script to align things like
# epicsEnvSet("ECM_IDLEPOLLPERIOD",   "0")
#


import re
import sys


RE_MATCH_EPICSENVSET = re.compile(r"(^\s*epicsEnvSet[^,]+,)\s*(.*)$")


def handle_epicsenvset(match_epicsenvset):
    gidx = 1
    part1 = match_epicsenvset.group(gidx)
    gidx += 1
    part2 = match_epicsenvset.group(gidx)
    print(f"{part1:30s} {part2}")


def main(argv=None):
    if not argv:
        argv = sys.argv

    for line in sys.stdin:
        line = line.strip()
        match_epicsenvset = RE_MATCH_EPICSENVSET.match(line)
        if match_epicsenvset is not None:
            handle_epicsenvset(match_epicsenvset)
            continue
        print(f"{line}")

    sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
