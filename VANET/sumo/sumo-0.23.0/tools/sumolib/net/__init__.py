"""
@file    __init__.py
@author  Daniel Krajzewicz
@author  Laura Bieker
@author  Karol Stosiek
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2008-03-27
@version $Id: __init__.py 18096 2015-03-17 09:50:59Z behrisch $

This file contains a content handler for parsing sumo network xml files.
It uses other classes from this module to represent the road network.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2015 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

from __future__ import print_function
import os
import sys
import math
from xml.sax import saxutils, parse, handler
from copy import copy
from itertools import *

import sumolib
from . import lane, edge, node, connection, roundabout
from .lane import Lane
from .edge import Edge
from .node import Node
from .connection import Connection
from .roundabout import Roundabout


class TLS:

    """Traffic Light Signal for a sumo network"""

    def __init__(self, id):
        self._id = id
        self._connections = []
        self._maxConnectionNo = -1
        self._programs = {}

    def addConnection(self, inLane, outLane, linkNo):
        self._connections.append([inLane, outLane, linkNo])
        if linkNo > self._maxConnectionNo:
            self._maxConnectionNo = linkNo

    def getConnections(self):
        return self._connections

    def getID(self):
        return self._id

    def getLinks(self):
        links = {}
        for connection in self._connections:
            if connection[2] not in links:
                links[connection[2]] = []
            links[connection[2]].append(connection)
        return links

    def getEdges(self):
        edges = set()
        for c in self._connections:
            edges.add(c[0].getEdge())
        return edges

    def addProgram(self, program):
        self._programs[program._id] = program

    def toXML(self):
        ret = ""
        for p in self._programs:
            ret = ret + self._programs[p].toXML(self._id)
        return ret


class TLSProgram:

    def __init__(self, id, offset, type):
        self._id = id
        self._type = type
        self._offset = offset
        self._phases = []

    def addPhase(self, state, duration):
        self._phases.append((state, duration))

    def toXML(self, tlsID):
        ret = '  <tlLogic id="%s" type="%s" programID="%s" offset="%s">\n' % (
            tlsID, self._type, self._id, self._offset)
        for p in self._phases:
            ret = ret + \
                '    <phase duration="%s" state="%s"/>\n' % (p[1], p[0])
        ret = ret + '  </tlLogic>\n'
        return ret


class Net:

    """The whole sumo network."""

    def __init__(self):
        self._location = {}
        self._id2node = {}
        self._id2edge = {}
        self._crossings_and_walkingAreas = set()
        self._id2tls = {}
        self._nodes = []
        self._edges = []
        self._tlss = []
        self._ranges = [[10000, -10000], [10000, -10000]]
        self._roundabouts = []
        self._rtree = None
        self._allLanes = []

    def setLocation(self, netOffset, convBoundary, origBoundary, projParameter):
        self._location["netOffset"] = netOffset
        self._location["convBoundary"] = convBoundary
        self._location["origBoundary"] = origBoundary
        self._location["projParameter"] = projParameter

    def addNode(self, id, type=None, coord=None, incLanes=None):
        if id not in self._id2node:
            n = node.Node(id, type, coord, incLanes)
            self._nodes.append(n)
            self._id2node[id] = n
        self.setAdditionalNodeInfo(self._id2node[id], type, coord, incLanes)
        return self._id2node[id]

    def setAdditionalNodeInfo(self, node, type, coord, incLanes):
        if coord != None and node._coord == None:
            node._coord = coord
            self._ranges[0][0] = min(self._ranges[0][0], coord[0])
            self._ranges[0][1] = max(self._ranges[0][1], coord[0])
            self._ranges[1][0] = min(self._ranges[1][0], coord[1])
            self._ranges[1][1] = max(self._ranges[1][1], coord[1])
        if incLanes != None and node._incLanes == None:
            node._incLanes = incLanes
        if type != None and node._type == None:
            node._type = type

    def addEdge(self, id, fromID, toID, prio, function, name):
        if id not in self._id2edge:
            fromN = self.addNode(fromID)
            toN = self.addNode(toID)
            e = edge.Edge(id, fromN, toN, prio, function, name)
            self._edges.append(e)
            self._id2edge[id] = e
        return self._id2edge[id]

    def addLane(self, edge, speed, length, allow=None, disallow=None):
        return lane.Lane(edge, speed, length, allow, disallow)

    def addRoundabout(self, nodes):
        r = roundabout.Roundabout(nodes)
        self._roundabouts.append(r)
        return r

    def addConnection(self, fromEdge, toEdge, fromlane, tolane, direction, tls, tllink):
        conn = connection.Connection(
            fromEdge, toEdge, fromlane, tolane, direction, tls, tllink)
        fromEdge.addOutgoing(conn)
        fromlane.addOutgoing(conn)
        toEdge._addIncoming(conn)

    def getEdges(self):
        return self._edges

    def getRoundabouts(self):
        return self._roundabouts

    def hasEdge(self, id):
        return id in self._id2edge

    def getEdge(self, id):
        return self._id2edge[id]

    def _initRTree(self, shapeList, includeJunctions=True):
        import rtree
        self._rtree = rtree.index.Index()
        self._rtree.interleaved = True
        for ri, shape in enumerate(shapeList):
            self._rtree.add(ri, shape.getBoundingBox(includeJunctions))

    def getNeighboringEdges(self, x, y, r=0.1, includeJunctions=True):
        edges = []
        try:
            if self._rtree == None:
                self._initRTree(self._edges, includeJunctions)
            for i in self._rtree.intersection((x - r, y - r, x + r, y + r)):
                e = self._edges[i]
                d = sumolib.geomhelper.distancePointToPolygon(
                    (x, y), e.getShape(includeJunctions))
                if d < r:
                    edges.append((e, d))
        except ImportError:
            for edge in self._edges:
                d = sumolib.geomhelper.distancePointToPolygon(
                    (x, y), edge.getShape(includeJunctions))
                if d < r:
                    edges.append((edge, d))
        return edges

    def getNeighboringLanes(self, x, y, r=0.1, includeJunctions=True):
        lanes = []
        try:
            if self._rtree == None:
                if not self._allLanes:
                    for edge in self._edges:
                        self._allLanes += edge.getLanes()
                self._initRTree(self._allLanes, includeJunctions)
            for i in self._rtree.intersection((x - r, y - r, x + r, y + r)):
                l = self._allLanes[i]
                d = sumolib.geomhelper.distancePointToPolygon(
                    (x, y), l.getShape(includeJunctions))
                if d < r:
                    lanes.append((l, d))
        except ImportError:
            for edge in self._edges:
                for l in edge.getLanes():
                    d = sumolib.geomhelper.distancePointToPolygon(
                        (x, y), l.getShape(includeJunctions))
                    if d < r:
                        lanes.append((l, d))
        return lanes

    def hasNode(self, id):
        return id in self._id2node

    def getNode(self, id):
        return self._id2node[id]

    def getNodes(self):
        return self._nodes

    def getTLSSecure(self, tlid):
        if tlid in self._id2tls:
            tls = self._id2tls[tlid]
        else:
            tls = TLS(tlid)
            self._id2tls[tlid] = tls
            self._tlss.append(tls)
        return tls

    def addTLS(self, tlid, inLane, outLane, linkNo):
        tls = self.getTLSSecure(tlid)
        tls.addConnection(inLane, outLane, linkNo)
        return tls

    def addTLSProgram(self, tlid, programID, offset, type):
        tls = self.getTLSSecure(tlid)
        program = TLSProgram(programID, offset, type)
        tls.addProgram(program)
        return program

    def setFoes(self, junctionID, index, foes, prohibits):
        self._id2node[junctionID].setFoes(index, foes, prohibits)

    def forbids(self, possProhibitor, possProhibited):
        return possProhibitor[0].getEdge()._to.forbids(possProhibitor, possProhibited)

    def getDownstreamEdges(self, edge, distance, stopOnTLS):
        ret = []
        seen = set()
        toProc = []
        toProc.append([edge, 0, []])
        while not len(toProc) == 0:
            ie = toProc.pop()
            if ie[0] in seen:
                continue
            seen.add(ie[0])
            if ie[1] + ie[0].getLength() >= distance:
                ret.append(
                    [ie[0], ie[0].getLength() + ie[1] - distance, ie[2], False])
                continue
            if len(ie[0]._incoming) == 0:
                ret.append([ie[0], ie[0].getLength() + ie[1], ie[2], True])
                continue
            mn = []
            hadTLS = False
            for ci in ie[0]._incoming:
                if ci not in seen:
                    prev = copy(ie[2])
                    if stopOnTLS and ci._tls and ci != edge and not hadTLS:
                        ret.append([ie[0], ie[1], prev, True])
                        hadTLS = True
                    else:
                        prev.append(ie[0])
                        mn.append([ci, ie[0].getLength() + ie[1], prev])
            if not hadTLS:
                toProc.extend(mn)
        return ret

    # the diagonal of the bounding box of all nodes
    def getBBoxDiameter(self):
        return math.sqrt(
            (self._ranges[0][0] - self._ranges[0][1]) ** 2 +
            (self._ranges[1][0] - self._ranges[1][1]) ** 2)

    def getGeoProj(self):
        import pyproj
        p1 = self._location["projParameter"].split()
        params = {}
        for p in p1:
            ps = p.split("=")
            if len(ps) == 2:
                params[ps[0]] = ps[1]
            else:
                params[ps[0]] = True
        return pyproj.Proj(projparams=params)

    def getLocationOffset(self):
        """ offset to be added after converting from geo-coordinates to UTM"""
        return list(map(float, self._location["netOffset"].split(",")))

    def convertLonLat2XY(self, lon, lat, rawUTM=False):
        x, y = self.getGeoProj()(lon, lat)
        if rawUTM:
            return x, y
        else:
            x_off, y_off = self.getLocationOffset()
            return x + x_off, y + y_off

    def convertXY2LonLat(self, x, y, rawUTM=False):
        if not rawUTM:
            x_off, y_off = self.getLocationOffset()
            x -= x_off
            y -= y_off
        return self.getGeoProj()(x, y, inverse=True)

    def move(self, dx, dy):
        for n in self._nodes:
            n._coord = (n._coord[0] + dx, n._coord[1] + dy)
        for e in self._edges:
            for l in e._lanes:
                l._shape = [(p[0] + dx, p[1] + dy) for p in l._shape]
            e.rebuildShape()


class NetReader(handler.ContentHandler):

    """Reads a network, storing the edge geometries, lane numbers and max. speeds"""

    def __init__(self, **others):
        self._net = others.get('net', Net())
        self._currentEdge = None
        self._currentNode = None
        self._currentLane = None
        self._currentShape = ""
        self._withPhases = others.get('withPrograms', False)
        self._withConnections = others.get('withConnections', True)
        self._withFoes = others.get('withFoes', True)

    def startElement(self, name, attrs):
        if name == 'location':
            self._net.setLocation(attrs["netOffset"], attrs["convBoundary"], attrs[
                                  "origBoundary"], attrs["projParameter"])
        if name == 'edge':
            function = attrs.get('function', '')
            if function == '':
                prio = -1
                if attrs.has_key('priority'):
                    prio = int(attrs['priority'])
                name = ""
                if attrs.has_key('name'):
                    name = attrs['name']
                self._currentEdge = self._net.addEdge(attrs['id'],
                                                      attrs['from'], attrs['to'], prio, function, name)
                if attrs.has_key('shape'):
                    self.processShape(self._currentEdge, attrs['shape'])
            else:
                if function in ['crossing', 'walkingarea']:
                    self._net._crossings_and_walkingAreas.add(attrs['id'])
                self._currentEdge = None
        if name == 'lane' and self._currentEdge != None:
            self._currentLane = self._net.addLane(
                self._currentEdge,
                float(attrs['speed']),
                float(attrs['length']),
                attrs.get('allow'),
                attrs.get('disallow'))
            if attrs.has_key('shape'):
                # deprecated: at some time, this is mandatory
                self._currentShape = attrs['shape']
            else:
                self._currentShape = ""
        if name == 'junction':
            if attrs['id'][0] != ':':
                self._currentNode = self._net.addNode(attrs['id'], attrs['type'], (float(
                    attrs['x']), float(attrs['y'])), attrs['incLanes'].split(" "))
        if name == 'succ' and self._withConnections:  # deprecated
            if attrs['edge'][0] != ':':
                self._currentEdge = self._net.getEdge(attrs['edge'])
                self._currentLane = attrs['lane']
                self._currentLane = int(
                    self._currentLane[self._currentLane.rfind('_') + 1:])
            else:
                self._currentEdge = None
        if name == 'succlane' and self._withConnections:  # deprecated
            lid = attrs['lane']
            if lid[0] != ':' and lid != "SUMO_NO_DESTINATION" and self._currentEdge:
                connected = self._net.getEdge(lid[:lid.rfind('_')])
                tolane = int(lid[lid.rfind('_') + 1:])
                if attrs.has_key('tl') and attrs['tl'] != "":
                    tl = attrs['tl']
                    tllink = int(attrs['linkIdx'])
                    tlid = attrs['tl']
                    toEdge = self._net.getEdge(lid[:lid.rfind('_')])
                    tolane2 = toEdge._lanes[tolane]
                    tls = self._net.addTLS(
                        tlid, self._currentEdge._lanes[self._currentLane], tolane2, tllink)
                    self._currentEdge.setTLS(tls)
                else:
                    tl = ""
                    tllink = -1
                toEdge = self._net.getEdge(lid[:lid.rfind('_')])
                tolane = toEdge._lanes[tolane]
                self._net.addConnection(self._currentEdge, connected, self._currentEdge._lanes[
                                        self._currentLane], tolane, attrs['dir'], tl, tllink)
        if name == 'connection' and self._withConnections and attrs['from'][0] != ":":
            fromEdgeID = attrs['from']
            toEdgeID = attrs['to']
            if not (fromEdgeID in self._net._crossings_and_walkingAreas or toEdgeID in
                    self._net._crossings_and_walkingAreas):
                fromEdge = self._net.getEdge(fromEdgeID)
                toEdge = self._net.getEdge(toEdgeID)
                fromLane = fromEdge.getLane(int(attrs['fromLane']))
                toLane = toEdge.getLane(int(attrs['toLane']))
                if attrs.has_key('tl') and attrs['tl'] != "":
                    tl = attrs['tl']
                    tllink = int(attrs['linkIndex'])
                    tls = self._net.addTLS(tl, fromLane, toLane, tllink)
                    fromEdge.setTLS(tls)
                else:
                    tl = ""
                    tllink = -1
                self._net.addConnection(
                    fromEdge, toEdge, fromLane, toLane, attrs['dir'], tl, tllink)
        # 'row-logic' is deprecated!!!
        if self._withFoes and name == 'ROWLogic':
            self._currentNode = attrs['id']
        if name == 'logicitem' and self._withFoes:  # deprecated
            self._net.setFoes(
                self._currentNode, int(attrs['request']), attrs["foes"], attrs["response"])
        if name == 'request' and self._withFoes:
            self._currentNode.setFoes(
                int(attrs['index']), attrs["foes"], attrs["response"])
        if self._withPhases and name == 'tlLogic':  # tl-logic is deprecated!!!
            self._currentProgram = self._net.addTLSProgram(
                attrs['id'], attrs['programID'], int(attrs['offset']), attrs['type'])
        if self._withPhases and name == 'phase':
            self._currentProgram.addPhase(
                attrs['state'], int(attrs['duration']))
        if name == 'roundabout':
            self._net.addRoundabout(attrs['nodes'].split())
        if name == 'param':
            if self._currentLane != None:
                self._currentLane._params[attrs['key']] = attrs['value']

    def characters(self, content):
        if self._currentLane != None:
            self._currentShape = self._currentShape + content

    def endElement(self, name):
        if name == 'lane':
            if self._currentLane:
                self.processShape(self._currentLane, self._currentShape)
                self._currentShape = ""
            self._currentLane = None
        if name == 'edge':
            if self._currentEdge and self._currentEdge._shape is None:
                self._currentEdge.rebuildShape()
            self._currentEdge = None
        # 'row-logic' is deprecated!!!
        if name == 'ROWLogic' or name == 'row-logic':
            self._haveROWLogic = False
        # tl-logic is deprecated!!!
        if self._withPhases and (name == 'tlLogic' or name == 'tl-logic'):
            self._currentProgram = None

    def processShape(self, object, shapeString):
        cshape = []
        es = shapeString.rstrip().split(" ")
        for e in es:
            p = e.split(",")
            cshape.append((float(p[0]), float(p[1])))
        object.setShape(cshape)

    def getNet(self):
        return self._net


def readNet(filename, **others):
    netreader = NetReader(**others)
    try:
        if not os.path.isfile(filename):
            print("Network file '%s' not found" % filename, file=sys.stderr)
            sys.exit(1)
        parse(filename, netreader)
    except None:
        print(
            "Please mind that the network format has changed in 0.13.0, you may need to update your network!", file=sys.stderr)
        sys.exit(1)
    return netreader.getNet()
