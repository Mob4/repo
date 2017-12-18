/****************************************************************************/
/// @file    OutputDevice_String.cpp
/// @author  Michael Behrisch
/// @date    2009
/// @version $Id: OutputDevice_String.cpp 18095 2015-03-17 09:39:00Z behrisch $
///
// An output device that encapsulates a stringstream
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2009-2015 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <sstream>
#include <string>
#include "OutputDevice_String.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
OutputDevice_String::OutputDevice_String(const bool binary, const unsigned int defaultIndentation)
    : OutputDevice(binary, defaultIndentation) {
}


OutputDevice_String::~OutputDevice_String() {
}


std::string
OutputDevice_String::getString() {
    return myStream.str();
}


std::ostream&
OutputDevice_String::getOStream() {
    return myStream;
}


/****************************************************************************/
