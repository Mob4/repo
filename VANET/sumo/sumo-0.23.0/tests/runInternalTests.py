#!/usr/bin/env python
"""
@file    runInternalTests.py
@author  Michael Behrisch
@date    2012-03-29
@version $Id: runInternalTests.py 18096 2015-03-17 09:50:59Z behrisch $


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2015 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

import optparse
import os
import subprocess
import sys
try:
    import texttestlib
    haveTextTestLib = True
except ImportError:
    haveTextTestLib = False


def runInternal(suffix, args, out=sys.stdout, gui=False):
    if os.name != "posix":
        suffix += ".exe"
    env = os.environ
    root = os.path.abspath(os.path.dirname(__file__))
    env["TEXTTEST_HOME"] = root
    env["ACTIVITYGEN_BINARY"] = os.path.join(
        root, "..", "bin", "activitygenInt" + suffix)
    env["DFROUTER_BINARY"] = os.path.join(
        root, "..", "bin", "dfrouterInt" + suffix)
    env["DUAROUTER_BINARY"] = os.path.join(
        root, "..", "bin", "duarouterInt" + suffix)
    env["JTRROUTER_BINARY"] = os.path.join(
        root, "..", "bin", "jtrrouterInt" + suffix)
    env["NETCONVERT_BINARY"] = os.path.join(
        root, "..", "bin", "netconvertInt" + suffix)
    env["NETGENERATE_BINARY"] = os.path.join(
        root, "..", "bin", "netgenerateInt" + suffix)
    env["OD2TRIPS_BINARY"] = os.path.join(
        root, "..", "bin", "od2tripsInt" + suffix)
    env["SUMO_BINARY"] = os.path.join(root, "..", "bin", "meso" + suffix)
    env["POLYCONVERT_BINARY"] = os.path.join(
        root, "..", "bin", "polyconvertInt" + suffix)
    env["GUISIM_BINARY"] = os.path.join(root, "..", "bin", "meso-gui" + suffix)
    env["MAROUTER_BINARY"] = os.path.join(
        root, "..", "bin", "marouter" + suffix)
    ttBin = 'texttest.py'
    if os.name == "posix":
        if subprocess.call(['which', 'texttest']) == 0:
            ttBin = 'texttest'
    elif haveTextTestLib:
        ttBin += "w"
    apps = "sumo.internal,sumo.meso,complex.meso,duarouter.astar,duarouter.chrouter"
    if gui:
        apps = "sumo.gui"
    subprocess.call("%s %s -a %s" %
                    (ttBin, args, apps), stdout=out, stderr=out, shell=True)

if __name__ == "__main__":
    optParser = optparse.OptionParser()
    optParser.add_option(
        "-s", "--suffix", default="", help="suffix to the fileprefix")
    optParser.add_option(
        "-g", "--gui", default=False, action="store_true", help="run gui tests")
    (options, args) = optParser.parse_args()
    runInternal(options.suffix, " ".join(
        ["-" + a for a in args]), gui=options.gui)
