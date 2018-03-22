#!/usr/bin/env python

from optparse import OptionParser
import os
import re
import sys

usage = "usage:   %prog {--shifty|--shifty]\n"


##################################################################
def parse_command_line():
    parser = OptionParser(usage=usage)

    parser.add_option("", "--shifty",
                      help="shift the Y values",
                      metavar="Y", action="store", dest="shiftY",
                      type="int", default=0)

    #parser.add_option("", "--shiftx",
    #                  help="shift the X values",
    #                  metavar="X", action="store", dest="shiftX",
    #                  type="int", default=0)

    (options, args) = parser.parse_args()
    if args:
        parser.print_help()
        print()
        print("Error: Must supply not supply arguments")
        sys.exit(1)
    return options






def shiftXorY(options, line):
    matchY  = re.compile('^( *<y>)([0-9]+)(</y> *)$')

    isMatchY = matchY.match(line)
    if isMatchY != None:
        pfx  = matchY.sub(r'\1', line)
        yPos = int(matchY.sub(r'\2', line))
        sfx  = matchY.sub(r'\3', line)
        yPos += options.shiftY
        #return str(yPos) + sfx
        if pfx[-1] == '\n':
            pfx = pfx[0:-1]
        return pfx + str(yPos) + sfx
    else:
        return line

#

options = parse_command_line()


for lineIn in sys.stdin:
    lineOut = shiftXorY(options, lineIn)
    sys.stdout.write(lineOut)
