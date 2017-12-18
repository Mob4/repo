#!/usr/bin/env python
"""
@file    edgesInDistricts.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2007-07-26
@version $Id: edgesInDistricts.py 18096 2015-03-17 09:50:59Z behrisch $

Parsing a number of networks and taz (district) files with shapes
this script writes a taz file with all the edges which are inside
the relevant taz.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2007-2015 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
import sys
import math
from xml.sax import parse, handler
from optparse import OptionParser


def parseShape(shape):
    poses = shape.split()
    ret = []
    for pos in poses:
        xy = pos.split(",")
        ret.append((float(xy[0]), float(xy[1])))
    return ret


def getBoundingBox(shape):
    min = [1e400, 1e400]
    max = [-1e400, -1e400]
    for pos in shape:
        for idx in [0, 1]:
            if pos[idx] > max[idx]:
                max[idx] = pos[idx]
            if pos[idx] < min[idx]:
                min[idx] = pos[idx]
    return min, max


def angle2D(p1, p2):
    theta1 = math.atan2(p1[1], p1[0])
    theta2 = math.atan2(p2[1], p2[0])
    dtheta = theta2 - theta1
    while dtheta > math.pi:
        dtheta -= 2.0 * math.pi
    while dtheta < -math.pi:
        dtheta += 2.0 * math.pi
    return dtheta


def isWithin(pos, shape):
    angle = 0.
    for i in range(0, len(shape) - 1):
        p1 = ((shape[i][0] - pos[0]), (shape[i][1] - pos[1]))
        p2 = ((shape[i + 1][0] - pos[0]), (shape[i + 1][1] - pos[1]))
        angle = angle + angle2D(p1, p2)
    i = len(shape) - 1
    p1 = ((shape[i][0] - pos[0]), (shape[i][1] - pos[1]))
    p2 = ((shape[0][0] - pos[0]), (shape[0][1] - pos[1]))
    angle = angle + angle2D(p1, p2)
    return math.fabs(angle) >= math.pi


# written into the net. All members are "private".
class NetDistrictEdgeHandler(handler.ContentHandler):

    def __init__(self):
        self._haveDistrict = False
        self._parsingDistrictShape = False
        self._parsingLaneShape = False
        self._currentID = ""
        self._shape = ""
        self._districtShapes = {}
        self._districtBoxes = {}
        self._edgeShapes = {}
        self._districtEdges = {}
        self._edgeDistricts = {}
        self._invalidatedEdges = []
        self._edgeSpeeds = {}
        self._edgeLengths = {}
        self._numLanes = {}

    def startElement(self, name, attrs):
        if name == 'taz' or name == 'poly':
            self._haveDistrict = True
            self._currentID = attrs['id']
            self._districtEdges[self._currentID] = []
            if attrs.has_key('shape'):
                self._shape = attrs['shape']
        elif name == 'shape' and self._haveDistrict:
            self._parsingDistrictShape = True
        elif name == 'edge':
            self._currentID = attrs['id']
            self._edgeDistricts[self._currentID] = []
        elif name == 'lane':
            self._parsingLaneShape = True
            if self._currentID not in self._numLanes:
                self._edgeSpeeds[self._currentID] = float(attrs['speed'])
                self._edgeLengths[self._currentID] = float(attrs['length'])
                self._numLanes[self._currentID] = 1
                if attrs.has_key('shape'):
                    self._shape = attrs['shape']
            else:
                self._numLanes[self._currentID] += 1

    def characters(self, content):
        if self._parsingDistrictShape:
            self._shape += content
        if self._parsingLaneShape:
            self._shape += content

    def endElement(self, name):
        if name == 'taz' or name == 'poly':
            self._haveDistrict = False
            if self._shape != '':
                self._districtShapes[self._currentID] = parseShape(self._shape)
                self._districtBoxes[self._currentID] = getBoundingBox(
                    self._districtShapes[self._currentID])
                self._shape = ""
        elif name == 'shape' and self._haveDistrict:
            self._parsingDistrictShape = False
        elif name == 'lane':
            self._parsingLaneShape = False
            if self._shape != '':
                self._edgeShapes[self._currentID] = parseShape(self._shape)
                self._shape = ""

    def computeWithin(self, complete, maxspeed, assignFrom, verbose):
        for idx, (edge, shape) in enumerate(self._edgeShapes.iteritems()):
            if self._edgeSpeeds[edge] < maxspeed:
                if assignFrom:
                    min = max = shape[0]
                else:
                    min, max = getBoundingBox(shape)
                for district, dshape in self._districtShapes.iteritems():
                    dmin, dmax = self._districtBoxes[district]
                    if dmin[0] <= max[0] and dmin[1] <= max[1] and dmax[0] >= min[0] and dmax[1] >= min[1]:
                        if assignFrom:
                            if isWithin(shape[0], dshape):
                                self._districtEdges[district].append(edge)
                                self._edgeDistricts[edge].append(district)
                                break
                        else:
                            for pos in shape:
                                if isWithin(pos, dshape):
                                    self._districtEdges[district].append(edge)
                                    self._edgeDistricts[edge].append(district)
                                    break
            if verbose and idx % 100 == 0:
                sys.stdout.write("%s/%s\r" % (idx, len(self._edgeShapes)))
        if complete:
            for edge, districts in self._edgeDistricts.iteritems():
                if len(districts) > 1:
                    self._invalidatedEdges.append(edge)

    def getEdgeDistrictMap(self):
        result = {}
        for edge, districts in self._edgeDistricts.iteritems():
            if len(districts) == 1:
                result[edge] = districts[0]
        return result

    def writeResults(self, output, weighted):
        fd = open(output, "w")
        fd.write("<tazs>\n")
        for district, edges in self._districtEdges.iteritems():
            validEdgeCount = 0
            for edge in edges:
                if edge not in self._invalidatedEdges:
                    validEdgeCount += 1
            if validEdgeCount == 0:
                print "District '" + district + "' has no edges!"
            else:
                fd.write("    <taz id=\"" + district + "\">\n")
                for edge in edges:
                    if edge not in self._invalidatedEdges:
                        weight = 1.
                        if weighted:
                            weight = self._edgeSpeeds[
                                edge] * self._edgeLengths[edge]
                        fd.write(
                            "        <tazSource id=\"" + edge + "\" weight=\"" + str(weight) + "\"/>\n")
                        fd.write(
                            "        <tazSink id=\"" + edge + "\" weight=\"" + str(weight) + "\"/>\n")
                fd.write("    </taz>\n")
        fd.write("</tazs>\n")
        fd.close()

    def getTotalLength(self, edge):
        return self._edgeLengths[edge] * self._numLanes[edge]


if __name__ == "__main__":
    optParser = OptionParser()
    optParser.add_option("-v", "--verbose", action="store_true",
                         default=False, help="tell me what you are doing")
    optParser.add_option("-c", "--complete", action="store_true",
                         default=False, help="assign edges only if they are not in more than one district")
    optParser.add_option("-n", "--net-files", dest="netfiles",
                         help="read SUMO network(s) from FILE(s) (mandatory)", metavar="FILE")
    optParser.add_option("-o", "--output", default="districts.taz.xml",
                         help="write results to FILE (default: %default)", metavar="FILE")
    optParser.add_option("-m", "--max-speed", type="float", dest="maxspeed",
                         default=1000.0, help="use lanes where speed is not greater than this (m/s) (default: %default)")
    optParser.add_option("-w", "--weighted", action="store_true",
                         default=False, help="Weights sources/sinks by lane number and length")
    optParser.add_option("-f", "--assign-from", action="store_true",
                         default=False, help="Assign the edge always to the district where the \"from\" node is located")
    (options, args) = optParser.parse_args()
    if not options.netfiles:
        optParser.print_help()
        optParser.exit("Error! Providing networks is mandatory")

    reader = NetDistrictEdgeHandler()
    for netfile in options.netfiles.split(","):
        if options.verbose:
            print "Reading net '" + netfile + "'"
        parse(netfile, reader)
    if options.verbose:
        print "Calculating"
    reader.computeWithin(
        options.complete, options.maxspeed, options.assign_from, options.verbose)
    if options.verbose:
        print "Writing results"
    reader.writeResults(options.output, options.weighted)
