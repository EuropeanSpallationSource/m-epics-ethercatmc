#!/usr/bin/env python

from optparse import OptionParser
import re
import sys

usage = "usage:   %prog --shifty <y> --shiftn <n>\n"


##################################################################
def parse_command_line():
    parser = OptionParser(usage=usage)

    parser.add_option(
        "",
        "--shiftx",
        help="shift the X values",
        metavar="X",
        action="store",
        dest="shiftX",
        type="int",
        default=0,
    )
    parser.add_option(
        "",
        "--shifty",
        help="shift the Y values",
        metavar="Y",
        action="store",
        dest="shiftY",
        type="int",
        default=0,
    )

    parser.add_option(
        "",
        "--shiftn",
        help="shift the aux bit number",
        metavar="N",
        action="store",
        dest="shiftN",
        type="int",
        default=0,
    )

    (options, args) = parser.parse_args()
    if args:
        parser.print_help()
        print()
        print("Error: Must supply not supply arguments")
        sys.exit(1)

    return options

    parser.print_help()
    print()
    print("Error: Must supply an option")
    sys.exit(1)


def shiftXY_NNN(options, line):
    matchX = re.compile(r"^( *<x>)([0-9]+)(</x> *)$")
    matchY = re.compile(r"^( *<y>)([0-9]+)(</y> *)$")
    matchN = re.compile(r"(.*)(NNN)(.*)")

    isMatchX = matchX.match(line)
    isMatchY = matchY.match(line)
    isMatchN = matchN.match(line)

    if isMatchX is not None:
        pfx = matchX.sub(r"\1", line)
        xPos = int(matchX.sub(r"\2", line))
        sfx = matchX.sub(r"\3", line)
        xPos += options.shiftX
        if pfx[-1] == "\n":
            pfx = pfx[0:-1]
        line = pfx + str(xPos) + sfx

    if isMatchY is not None:
        pfx = matchY.sub(r"\1", line)
        yPos = int(matchY.sub(r"\2", line))
        sfx = matchY.sub(r"\3", line)
        yPos += options.shiftY
        if pfx[-1] == "\n":
            pfx = pfx[0:-1]
        line = pfx + str(yPos) + sfx

    if isMatchN is not None:
        pfm = matchN.sub(r"\1", line)
        # mPos = int(matchM.sub(r'\2', line))
        sfm = matchN.sub(r"\3", line)
        mPos = options.shiftN
        if pfm[-1] == "\n":
            pfm = pfm[0:-1]
        line = pfm + str(mPos) + sfm

    return line


#

options = parse_command_line()


for lineIn in sys.stdin:
    lineOut = shiftXY_NNN(options, lineIn)
    sys.stdout.write(lineOut)
