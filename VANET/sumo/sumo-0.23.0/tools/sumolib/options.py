"""
@file    options.py
@author  Jakob Erdmann
@author  Michael Behrisch
@date    2012-03-15
@version $Id: options.py 18096 2015-03-17 09:50:59Z behrisch $

Provides utility functions for dealing with program options

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2012-2015 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

from __future__ import print_function
import os
import sys
import subprocess
from collections import namedtuple
import re
from xml.sax import parse, handler


def get_long_option_names(application):
    # using option --save-template and parsing xml would be prettier
    # but we do not want to rely on a temporary file
    output, error = subprocess.Popen(
        [application, '--help'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE).communicate()
    reprog = re.compile('(--\S*)\s')
    result = []
    for line in output.split(os.linesep):
        m = reprog.search(line)
        if m:
            result.append(m.group(1))
    return result


Option = namedtuple("Option", ["name", "value", "type", "help"])


class OptionReader(handler.ContentHandler):

    """Reads an option file"""

    def __init__(self):
        self.opts = []

    def startElement(self, name, attrs):
        if attrs.has_key('value'):
            self.opts.append(
                Option(name, attrs['value'], attrs.get('type'), attrs.get('help')))


def readOptions(filename):
    optionReader = OptionReader()
    try:
        if not os.path.isfile(filename):
            print("Option file '%s' not found" % filename, file=sys.stderr)
            sys.exit(1)
        parse(filename, optionReader)
    except None:
        print("Invalid option file '%s'" % filename, file=sys.stderr)
        sys.exit(1)
    return optionReader.opts
