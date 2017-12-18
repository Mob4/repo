
#ifdef NS3_MODULE_COMPILATION
# error "Do not include ns3 module aggregator headers from other modules; these are meant only for end user scripts."
#endif

#ifndef NS3_MODULE_GEOLIB
    

// Module headers:
#include "AzimuthalEquidistant.hpp"
#include "CassiniSoldner.hpp"
#include "Constants.hpp"
#include "DMS.hpp"
#include "EllipticFunction.hpp"
#include "GeoCoords.hpp"
#include "Geocentric.hpp"
#include "Geodesic.hpp"
#include "Geoid.hpp"
#include "LambertConformalConic.hpp"
#include "LocalCartesian.hpp"
#include "MGRS.hpp"
#include "PolarStereographic.hpp"
#include "TransverseMercator.hpp"
#include "TransverseMercatorExact.hpp"
#include "UTMUPS.hpp"
#endif
