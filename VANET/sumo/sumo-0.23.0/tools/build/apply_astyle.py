#!/usr/bin/env python
"""
@file    apply_astyle.py
@author  Michael Behrisch
@author  Daniel Krajzewicz
@date    2007
@version $Id: apply_astyle.py 18096 2015-03-17 09:50:59Z behrisch $

Applies astyle with the proper settings used in SUMO on all
 files in src.

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
try:
    import autopep8
    autopep = True
except:
    autopep = False

srcRoot = os.path.join(os.path.dirname(__file__), "../../src/")
for root, dirs, files in os.walk(srcRoot):
    for name in files:
        if name.endswith(".h") or name.endswith(".cpp"):
            subprocess.call(
                "astyle --style=java --unpad-paren --pad-header --pad-oper --add-brackets --indent-switches --align-pointer=type -n".split() + [os.path.join(root, name)])
        for ignoreDir in ['.svn', 'foreign']:
            if ignoreDir in dirs:
                dirs.remove(ignoreDir)

if autopep:
    sumoRoot = os.path.join(os.path.dirname(__file__), "../../")
    for root, dirs, files in os.walk(sumoRoot):
        for name in files:
            if name.endswith(".py"):
                subprocess.call(
                    "autopep8 --in-place".split() + [os.path.join(root, name)])
        for ignoreDir in ['.svn', 'foreign']:
            if ignoreDir in dirs:
                dirs.remove(ignoreDir)
