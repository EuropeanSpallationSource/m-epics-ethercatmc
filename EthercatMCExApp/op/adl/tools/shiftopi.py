#!/usr/bin/env python

from optparse import OptionParser
import os
import re
import sys

usage = "usage:   %prog [--shifty|--shifty][--shiftm]\n"


##################################################################
def parse_command_line():
    parser = OptionParser(usage=usage)

    parser.add_option("", "--shifty",
                      help="shift the Y values",
                      metavar="Y", action="store", dest="shiftY",
                      type="int", default=0)

    parser.add_option("", "--shiftx",
                      help="shift the X values",
                      metavar="X", action="store", dest="shiftX",
                      type="int", default=0)

    parser.add_option("", "--shiftw",
                      help="shift the width",
                      metavar="W", action="store", dest="shiftW",
                      type="int", default=0)

    parser.add_option("", "--shifth",
                      help="shift the height",
                      metavar="H", action="store", dest="shiftH",
                      type="int", default=0)

    parser.add_option("", "--shiftm",
                      help="shift the M values",
                      metavar="M", action="store", dest="shiftM",
                      type="int", default=0)
    parser.add_option("", "--debug",
                      help="debug printouts",
                      metavar="debug", action="store", dest="debug",
                      type="int", default=0)

    (options, args) = parser.parse_args()
    if args:
        parser.print_help()
        print()
        print("Error: Must supply not supply arguments")
        sys.exit(1)

    #if options.shiftY != 0 and options.shiftX == 0:
    #    return options
    #if options.shiftY == 0 and options.shiftX != 0:
    #
    return options

    parser.print_help()
    print()
    print("Error: Must supply an option")
    sys.exit(1)



def shiftXorY(options, line):
    matchX  = re.compile('^([ \t]*x=)([0-9]+)(.*)$')
    matchY  = re.compile('^([ \t]*y=)([0-9]+)(.*)$')
    matchW  = re.compile('^([ \t]*width=)([0-9]+)(.*)$')
    matchH  = re.compile('^([ \t]*height=)([0-9]+)(.*)$')
    matchM  = re.compile('(.*\([MR])([0-9]+)(\).*)')
    matchPoint  = re.compile('(^[ \t]*\()([0-9]+)(,)([0-9]+)(\)[ \t]*$)')    

    isMatchX = matchX.match(line)
    isMatchM = matchM.match(line)
    isMatchW = matchW.match(line)
    isMatchH = matchH.match(line)
    isMatchY = matchY.match(line)
    isMatchPoint = matchPoint.match(line)

    if isMatchPoint != None:
        #if options.debug > 0:
        pfx  = matchPoint.sub(r'\1', line)
        xPos = int(matchPoint.sub(r'\2', line))
        comma = matchPoint.sub(r'\3', line)
        yPos = int(matchPoint.sub(r'\4', line))
        sfx  = matchPoint.sub(r'\5', line)
        xPos += options.shiftX
        yPos += options.shiftY
        # The literals seem to get an extra "\n", I don't know why
        # Remove it in any case
        if pfx[-1] == '\n':
            pfx = pfx[0:-1]
        if comma[-1] == '\n':
            comma = comma[0:-1]
        line = pfx + str(xPos) + comma  + str(yPos) + sfx
    
    if isMatchY != None:
        pfx  = matchY.sub(r'\1', line)
        yPos = int(matchY.sub(r'\2', line))
        sfx  = matchY.sub(r'\3', line)
        yPos += options.shiftY
        if pfx[-1] == '\n':
            pfx = pfx[0:-1]
        line = pfx + str(yPos) + sfx

    if isMatchX != None:
        pfx  = matchX.sub(r'\1', line)
        xPos = int(matchX.sub(r'\2', line))
        sfx  = matchX.sub(r'\3', line)
        xPos += options.shiftX
        if pfx[-1] == '\n':
            pfx = pfx[0:-1]
        line = pfx + str(xPos) + sfx

    if isMatchH != None:
        pfx  = matchH.sub(r'\1', line)
        yPos = int(matchH.sub(r'\2', line))
        sfx  = matchH.sub(r'\3', line)
        yPos += options.shiftH
        if pfx[-1] == '\n':
            pfx = pfx[0:-1]
        line = pfx + str(yPos) + sfx

    if isMatchW != None:
        pfx  = matchW.sub(r'\1', line)
        xPos = int(matchW.sub(r'\2', line))
        sfx  = matchW.sub(r'\3', line)
        xPos += options.shiftW
        if pfx[-1] == '\n':
            pfx = pfx[0:-1]
        line = pfx + str(xPos) + sfx

    if isMatchM != None:
        pfm  = matchM.sub(r'\1', line)
        mPos = int(matchM.sub(r'\2', line))
        sfm  = matchM.sub(r'\3', line)
        mPos += options.shiftM
        if pfm[-1] == '\n':
            pfm = pfm[0:-1]
        line = pfm + str(mPos) + sfm


    return line

#

options = parse_command_line()


for lineIn in sys.stdin:
    lineOut = shiftXorY(options, lineIn)
    sys.stdout.write(lineOut)
