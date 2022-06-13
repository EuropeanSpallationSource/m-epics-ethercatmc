#!/usr/bin/env python

from optparse import OptionParser
import os
import re
import sys

usage = "usage:   %prog [--shifty|--shifty][--shiftm]\n"


##################################################################
def parse_command_line():
    parser = OptionParser(usage=usage)

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
        "--shiftm",
        help="shift the M values",
        metavar="M",
        action="store",
        dest="shiftM",
        type="int",
        default=0,
    )

    (options, args) = parser.parse_args()
    if args:
        parser.print_help()
        print()
        print("Error: Must supply not supply arguments")
        sys.exit(1)

    # if options.shiftY != 0 and options.shiftX == 0:
    #    return options
    # if options.shiftY == 0 and options.shiftX != 0:
    #
    return options

    parser.print_help()
    print()
    print("Error: Must supply an option")
    sys.exit(1)


def shiftXorY(options, line):
    matchX = re.compile("^( *<x>)([0-9]+)(</x> *)$")
    matchY = re.compile("^( *<y>)([0-9]+)(</y> *)$")
    matchM = re.compile("(.*\([MR])([0-9]+)(\).*)")

    isMatchX = matchX.match(line)
    isMatchM = matchM.match(line)
    isMatchY = matchY.match(line)

    if isMatchY != None:
        pfx = matchY.sub(r"\1", line)
        yPos = int(matchY.sub(r"\2", line))
        sfx = matchY.sub(r"\3", line)
        yPos += options.shiftY
        if pfx[-1] == "\n":
            pfx = pfx[0:-1]
        line = pfx + str(yPos) + sfx

    if isMatchX != None:
        pfx = matchX.sub(r"\1", line)
        xPos = int(matchX.sub(r"\2", line))
        sfx = matchX.sub(r"\3", line)
        xPos += options.shiftX
        if pfx[-1] == "\n":
            pfx = pfx[0:-1]
        line = pfx + str(xPos) + sfx

    if isMatchM != None:
        pfm = matchM.sub(r"\1", line)
        mPos = int(matchM.sub(r"\2", line))
        sfm = matchM.sub(r"\3", line)
        mPos += options.shiftM
        if pfm[-1] == "\n":
            pfm = pfm[0:-1]
        line = pfm + str(mPos) + sfm

    return line


#

options = parse_command_line()


for lineIn in sys.stdin:
    lineOut = shiftXorY(options, lineIn)
    sys.stdout.write(lineOut)
