#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file    runner.py
@author  Jakob Erdmann
@author  Michael Behrisch
@date    2013-08-16
@version $Id: runner.py 18105 2015-03-19 07:39:56Z behrisch $

import sumo network then export to the given format and import again
and check for idempotency with the original sumo network

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2015 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""


import sys
import os
import subprocess
sys.path.append(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', "tools"))
import sumolib

format = sys.argv[1]
netconvert = sumolib.checkBinary('netconvert')

args1 = [netconvert,
         '--sumo-net-file', 'input_net.net.xml',
         '--%s-output' % format, format]

subprocess.call(args1)
subprocess.call([netconvert, '--%s' % format, format])
