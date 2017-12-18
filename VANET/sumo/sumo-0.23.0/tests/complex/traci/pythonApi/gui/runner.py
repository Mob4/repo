#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file    runner.py
@author  Michael Behrisch
@author  Daniel Krajzewicz
@date    2011-03-04
@version $Id: runner.py 18096 2015-03-17 09:50:59Z behrisch $


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2015 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

import os
import subprocess
import sys
import random
import struct
import random
import time
sys.path.append(os.path.join(
    os.path.dirname(sys.argv[0]), "..", "..", "..", "..", "..", "tools"))
import traci
import sumolib

sumoBinary = sumolib.checkBinary('sumo-gui')

PORT = sumolib.miscutils.getFreeSocketPort()
sumoProcess = subprocess.Popen(
    "%s -S -Q -c sumo.sumocfg --remote-port %s" % (sumoBinary, PORT), shell=True, stdout=sys.stdout)
traci.init(PORT)
for step in range(3):
    print "step", step
    traci.simulationStep()
time.sleep(1)  # give the gui a chance to draw itself
print "views", traci.gui.getIDList()
viewID = traci.gui.DEFAULT_VIEW
print "examining", viewID
print "zoom", traci.gui.getZoom(viewID)
print "offset", traci.gui.getOffset(viewID)
print "schema", traci.gui.getSchema(viewID)
print "visible boundary", traci.gui.getBoundary(viewID)

traci.gui.subscribe(viewID)
print traci.gui.getSubscriptionResults(viewID)
for step in range(3, 6):
    print "step", step
    traci.simulationStep()
    print traci.gui.getSubscriptionResults(viewID)
traci.close()
