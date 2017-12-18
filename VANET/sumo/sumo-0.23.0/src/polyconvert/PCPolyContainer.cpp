/****************************************************************************/
/// @file    PCPolyContainer.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Melanie Knocke
/// @date    Mon, 05 Dec 2005
/// @version $Id: PCPolyContainer.cpp 18137 2015-03-24 15:12:38Z behrisch $
///
// A storage for loaded polygons and pois
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2005-2015 DLR (http://www.dlr.de/) and contributors
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

#include <string>
#include <algorithm>
#include <map>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StringUtils.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/shapes/Polygon.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include "PCPolyContainer.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
PCPolyContainer::PCPolyContainer(bool prune,
                                 const Boundary& pruningBoundary,
                                 const std::vector<std::string>& removeByNames)
    : myPruningBoundary(pruningBoundary), myDoPrune(prune),
      myRemoveByNames(removeByNames) {}


PCPolyContainer::~PCPolyContainer() {
    clear();
}


bool
PCPolyContainer::insert(const std::string& id, Polygon* poly,
                        int layer, bool ignorePruning) {
    // check whether the polygon lies within the wished area
    //  - if such an area was given
    if (myDoPrune && !ignorePruning) {
        Boundary b = poly->getShape().getBoxBoundary();
        if (!b.partialWithin(myPruningBoundary)) {
            delete poly;
            return false;
        }
    }
    // check whether the polygon was named to be a removed one
    if (find(myRemoveByNames.begin(), myRemoveByNames.end(), id) != myRemoveByNames.end()) {
        delete poly;
        return false;
    }
    //
    PolyCont::iterator i = myPolyCont.find(id);
    if (i != myPolyCont.end()) {
        WRITE_ERROR("Polygon '" + id + "' could not be added.");
        delete poly;
        return false;
    }
    myPolyCont[id] = poly;
    myPolyLayerMap[poly] = layer;
    return true;
}


bool
PCPolyContainer::insert(const std::string& id, PointOfInterest* poi,
                        int layer, bool ignorePruning) {
    // check whether the poi lies within the wished area
    //  - if such an area was given
    if (myDoPrune && !ignorePruning) {
        if (!myPruningBoundary.around(*poi)) {
            delete poi;
            return false;
        }
    }
    // check whether the polygon was named to be a removed one
    if (find(myRemoveByNames.begin(), myRemoveByNames.end(), id) != myRemoveByNames.end()) {
        delete poi;
        return false;
    }
    //
    POICont::iterator i = myPOICont.find(id);
    if (i != myPOICont.end()) {
        WRITE_ERROR("POI '" + id + "' could not be added.");
        delete poi;
        return false;
    }
    myPOICont[id] = poi;
    myPOILayerMap[poi] = layer;
    return true;
}


bool
PCPolyContainer::containsPolygon(const std::string& id) {
    return myPolyCont.find(id) != myPolyCont.end();
}


void
PCPolyContainer::clear() {
    // polys
    for (PolyCont::iterator i = myPolyCont.begin(); i != myPolyCont.end(); i++) {
        delete(*i).second;
    }
    myPolyCont.clear();
    myPolyLayerMap.clear();
    // pois
    for (POICont::iterator i = myPOICont.begin(); i != myPOICont.end(); i++) {
        delete(*i).second;
    }
    myPOICont.clear();
    myPOILayerMap.clear();
}


void
PCPolyContainer::report() {
    WRITE_MESSAGE("   " + toString(getNoPolygons()) + " polygons loaded.");
    WRITE_MESSAGE("   " + toString(getNoPOIs()) + " pois loaded.");
}


void
PCPolyContainer::save(const std::string& file, bool useGeo) {
    const GeoConvHelper& gch = GeoConvHelper::getFinal();
    if (useGeo && !gch.usingGeoProjection()) {
        WRITE_WARNING("Ignoring option \"proj.plain-geo\" because no geo-conversion has been defined");
        useGeo = false;
    }
    OutputDevice& out = OutputDevice::getDevice(file);
    out.writeXMLHeader("additional", "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.dlr.de/xsd/additional_file.xsd\"");
    if (useGeo) {
        out.setPrecision(GEO_OUTPUT_ACCURACY);
    } else {
        GeoConvHelper::writeLocation(out);
    }
    // write polygons
    for (PolyCont::iterator i = myPolyCont.begin(); i != myPolyCont.end(); ++i) {
        i->second->writeXML(out, useGeo);
    }
    // write pois
    for (POICont::iterator i = myPOICont.begin(); i != myPOICont.end(); ++i) {
        PointOfInterest* p = i->second;
        out.openTag(SUMO_TAG_POI);
        out.writeAttr(SUMO_ATTR_ID, StringUtils::escapeXML(p->getID()));
        out.writeAttr(SUMO_ATTR_TYPE, StringUtils::escapeXML(p->getType()));
        out.writeAttr(SUMO_ATTR_COLOR, p->getColor());
        out.writeAttr(SUMO_ATTR_LAYER, p->getLayer());
        if (useGeo) {
            Position pos(*p);
            gch.cartesian2geo(pos);
            out.writeAttr(SUMO_ATTR_LON, pos.x());
            out.writeAttr(SUMO_ATTR_LAT, pos.y());
        } else {
            out.writeAttr(SUMO_ATTR_X, p->x());
            out.writeAttr(SUMO_ATTR_Y, p->y());
        }
        if (p->getAngle() != Shape::DEFAULT_ANGLE) {
            out.writeAttr(SUMO_ATTR_ANGLE, p->getAngle());
        }
        if (p->getImgFile() != Shape::DEFAULT_IMG_FILE) {
            out.writeAttr(SUMO_ATTR_IMGFILE, p->getImgFile());
        }
        if (p->getWidth() != Shape::DEFAULT_IMG_WIDTH) {
            out.writeAttr(SUMO_ATTR_WIDTH, p->getWidth());
        }
        if (p->getHeight() != Shape::DEFAULT_IMG_HEIGHT) {
            out.writeAttr(SUMO_ATTR_HEIGHT, p->getHeight());
        }
        for (std::map<std::string, std::string>::const_iterator j = p->getMap().begin(); j != p->getMap().end(); ++j) {
            out.openTag(SUMO_TAG_PARAM);
            out.writeAttr(SUMO_ATTR_KEY, (*j).first);
            out.writeAttr(SUMO_ATTR_VALUE, (*j).second);
            out.closeTag();
        }
        out.closeTag();
    }
    out.close();
}


int
PCPolyContainer::getEnumIDFor(const std::string& key) {
    if (myIDEnums.find(key) == myIDEnums.end()) {
        myIDEnums[key] = 0;
        return 0;
    } else {
        myIDEnums[key] = myIDEnums[key] + 1;
        return myIDEnums[key];
    }
}



/****************************************************************************/

