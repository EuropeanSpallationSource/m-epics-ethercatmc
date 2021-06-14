#!/usr/bin/env python

from optparse import OptionParser
import os
import re
import sys

usage = "usage:   %prog --shifty <y> --shiftn <n>\n"


##################################################################
def parse_command_line():
    parser = OptionParser(usage=usage)

    parser.add_option("", "--shifty",
                      help="shift the Y values",
                      metavar="Y", action="store", dest="shiftY",
                      type="int", default=0)

    parser.add_option("", "--shiftn",
                      help="shift the aux bit number",
                      metavar="N", action="store", dest="shiftN",
                      type="int", default=0)

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



def shiftY_NNN(options, line):
    matchY  = re.compile('^( *<y>)([0-9]+)(</y> *)$')
    matchN  = re.compile('(.*)(NNN)(.*)')

    isMatchY = matchY.match(line)
    isMatchN = matchN.match(line)

    if isMatchY != None:
        pfx  = matchY.sub(r'\1', line)
        yPos = int(matchY.sub(r'\2', line))
        sfx  = matchY.sub(r'\3', line)
        yPos += options.shiftY
        if pfx[-1] == '\n':
            pfx = pfx[0:-1]
        line = pfx + str(yPos) + sfx

    if isMatchN != None:
        pfm  = matchN.sub(r'\1', line)
        #mPos = int(matchM.sub(r'\2', line))
        sfm  = matchN.sub(r'\3', line)
        mPos = options.shiftN
        if pfm[-1] == '\n':
            pfm = pfm[0:-1]
        line = pfm + str(mPos) + sfm

    return line

#

options = parse_command_line()


for lineIn in sys.stdin:
    lineOut = shiftY_NNN(options, lineIn)
    sys.stdout.write(lineOut)
