#!/bin/bash

# ###############################################################
# Check parameters enter by user
checkParameters() {

    OSMInput=$1
    NODES=$2
    PREFIX=$3
    SUMOHOME=$4
    POLYTYPE=$5
    exact=$6
    NoSumoGui=$7

    error=0

    if [ ! -f "$OSMInput" ]; then
	echo "--> Error: $OSMInput not existing"
	error=1
    fi 

    if [ ! -d "$SUMOHOME" ]; then
	if [ ! -d "$SUMO_HOME" ]; then
	    if [ -d /usr/share/sumo ]; then
		export SUMO_HOME=/usr/share/sumo
	    else
		echo "--> Error: Please check sumo home and its files (SUMO home directory not existing)"
		error=1
	    fi
	fi
    else 
	export SUMO_HOME=$SUMOHOME
    fi
    
    if [ ! -f "$POLYTYPE" ]; then
	echo "--> Error: Please check polygone configuration file to be used (file $POLYTYPE not existing)"
	error=1
    fi 
    if [ $NODES -le 0 ]; then
	echo "--> Error: invalid number of nodes ($NODES), should be greater than 0"
	error=1
    fi 
    if [ -f "$exact" ]; then
	echo "--> A script will be applied on result to get the exact number of nodes specified in nodes (ie. $NODES)"
    fi

    if [ $error -eq 1 ]; then
	echo -n "Press <Enter> to display help"
	read y
	usage
    fi
}

# ###############################################################
# Generation of NS2 mobility 
# main($OSMInput,$NODES,$PREFIX,$SUMOHOME,$POLYTYPE);
mainStart() {
    OSMInput=$1
    NODES=$2
    PREFIX=$3
    SUMOHOME=$4
    POLYTYPE=$5
    exact=$6
    NoSumoGui=$7

    checkParameters $OSMInput $NODES $PREFIX $SUMOHOME $POLYTYPE $exact $NoSumoGui

    echo "Starting the generation for file $OSMInput"

    # Since a certain number of nodes will be removed => add 50% nodes then the script $exact (if specified) will remove them
    let NODESESTIMATION=$NODES+$NODES*5/10

    # File names
    NETFILE=$PREFIX.net.xml
    POLYFILE=$PREFIX.poly.xml
    ROUFILE=$PREFIX.rou.xml
    SUMOCFGFILE=$PREFIX.sumo.cfg
    FCDOUTPUTFILE=$PREFIX.fcdoutput.xml
    NS2OUTPUTFILE=$PREFIX$NODESESTIMATION.ns2mobility.xml
    NETSTATE=$PREFIX$NODESESTIMATION.state.xml

    # Create a net file from OSM file
    echo ">> $SUMOHOME/bin/netconvert --osm-files $OSMInput -o $NETFILE"
    $SUMOHOME/bin/netconvert --osm-files $OSMInput -o $NETFILE 2>netconvert.errors.txt
    if [ $? != 0 ]; then echo "Error in netconvert"; exit 1; fi
    if [ -f netconvert.errors.txt -a -s netconvert.errors.txt ]; then
	echo "Some warnings occurs during the generation of net file, see netconvert.errors.txt for details"
    fi
    echo "Net file was generated"

    # Create the polygon files & generate sbg.poly.xml
    echo "$SUMOHOME/bin/polyconvert --net-file $NETFILE --osm-files $OSMInput --type-file=$POLYTYPE -o $POLYFILE"
    $SUMOHOME/bin/polyconvert --net-file $NETFILE --osm-files $OSMInput --type-file=$POLYTYPE -o $POLYFILE
    if [ $? != 0 ]; then echo "Error in polyconvert"; exit 1; fi
    echo "Polygones were generated"

    # Create random trips (end simulation time = number of nodes) & Generate trips.trips.xml
    echo ">> $SUMOHOME/tools/trip/randomTrips.py -n $NETFILE -b 0 -e $NODESESTIMATION"
    $SUMOHOME/tools/trip/randomTrips.py -n $NETFILE -b 0 -e $NODESESTIMATION
    if [ $? != 0 ]; then echo "Error in randomTrips.py"; exit 1; fi
    echo "Random trips for $NODESESTIMATION s was generated with same number of nodes"

    # Generate route files
    echo ">> $SUMOHOME/bin/duarouter -n $NETFILE -t trips.trips.xml -o $ROUFILE --ignore-errors"
    $SUMOHOME/bin/duarouter -n $NETFILE -t trips.trips.xml -o $ROUFILE --ignore-errors 2>duarouter.errors.txt
    if [ $? != 0 ]; then echo "Error in duarouter"; exit 1; fi
    if [ -f duarouter.errors.txt -a -s duarouter.errors.txt ]; then
	echo "Some warnings occurs during the generation of route file, see duarouter.errors.txt for details"
    fi
    echo "Vehicles generated"

    # Creation du fichier CFG de sumo (sbg.sumo.cfg)
    echo "<configuration>" > $SUMOCFGFILE
    echo "    <input>" >> $SUMOCFGFILE
    echo "        <net-file value=\"$NETFILE\"/>" >> $SUMOCFGFILE
    echo "	<route-files value=\"$ROUFILE\"/>" >> $SUMOCFGFILE
    echo "	<additional-files value=\"$POLYFILE\"/>" >> $SUMOCFGFILE
    echo "    </input>" >> $SUMOCFGFILE
    echo "    <time>" >> $SUMOCFGFILE
    echo "	<begin value=\"0\"/>" >> $SUMOCFGFILE
    echo "	<end value=\"$NODESESTIMATION\"/>" >> $SUMOCFGFILE
    echo "    </time>" >> $SUMOCFGFILE
    echo "</configuration>" >> $SUMOCFGFILE

    # Display with sumo-gui the configuration done
    if [ $NoSumoGui -ne 1 ]; then
	echo "Please quit sumo-gui to continue the generation in NS2 mobility file"
	$SUMOHOME/bin/sumo-gui -c $SUMOCFGFILE > /dev/null 2>/dev/null
    fi
    echo ">> $SUMOHOME/bin/sumo -c $SUMOCFGFILE --fcd-output $FCDOUTPUTFILE --netstate-dump $NETSTATE"
    $SUMOHOME/bin/sumo -c $SUMOCFGFILE --fcd-output $FCDOUTPUTFILE --netstate-dump $NETSTATE > /dev/null 2> /dev/null
    if [ $? != 0 ]; then echo "Error in sumo"; exit 1; fi
    echo "Conversion to FCD output done"

    # Scripts for NS2 conversion
    if [ -f $SUMOHOME/tools/traceExporter.py ]; then
	echo "Be sure that you have the latest version of traceExporter.py following the bug report here: http://sumo-sim.org/trac.wsgi/browser/trunk/sumo/tools/bin/traceExporter.py?rev=15228"
	echo ">>  $SUMOHOME/tools/traceExporter.py --fcd-input $FCDOUTPUTFILE --ns2config-output $PREFIX$NODESESTIMATION.config.tcl --ns2activity-output $PREFIX$NODESESTIMATION.activity.tcl --ns2mobility-output $NS2OUTPUTFILE --penetration 1 --begin 0 --end $NODESESTIMATION"
	$SUMOHOME/tools/traceExporter.py --fcd-input $FCDOUTPUTFILE --ns2config-output $PREFIX$NODESESTIMATION.config.tcl --ns2activity-output $PREFIX$NODESESTIMATION.activity.tcl --ns2mobility-output $NS2OUTPUTFILE --penetration 1 --begin 0 --end $NODESESTIMATION
	# echo reduce the number of nodes based on a script in frederic/scripts
	if [ -f $exact ]; then
	    a=`$exact -input=$NS2OUTPUTFILE -output=$PREFIX$NODES.mobility.tcl -nbnodes=$NODES -duration=$NODESESTIMATION`
	    echo $a
	    if [ $? != 0 ]; then
		echo "Error: script $exact gives $a"
	    else
		nb=`grep "# Found" $PREFIX$NODES.mobility.tcl | cut -d' ' -f 3`
		# Display			
		echo "NS2 File $NS2OUTPUTFILE generated with $nb nodes !"
	    fi
	else
	    # Add number of nodes at beginning		
	    nb=`grep "$node_(" $NS2OUTPUTFILE | grep X_ | cut -d'(' -f 2 | cut -d ')' -f 1 | sort -nu | wc -l`
	    echo "# Found $nb nodes" > /tmp/$NS2OUTPUTFILE
	    cat $NS2OUTPUTFILE >> /tmp/$NS2OUTPUTFILE
	    cp /tmp/$NS2OUTPUTFILE $PREFIX$nb.ns2mobility.xml
	    # Display
	    echo "NS2 File $NS2OUTPUTFILE generated with $nb nodes !"
	fi
	# else
	
	# 	if [ -f  /usr/share/sumo/tools/bin/traceExporter.jar ]; then
	# 		echo "--> Warning: Using deprecated software traceExporter.jar to generate ns2 scenario"
	# 		java -jar $SUMOHOME/tools/traceExporter/traceExporter.jar ns2 -n $NETFILE -t $NETSTATE -a $PREFIX$NODES.activity.tcl -m $NS2OUTPUTFILE -c $PREFIX$NODES.config.tcl -p 1 -b 0 -e $NODESESTIMATION
	# 		nb=`grep "# SUMO-ID:" $NS2OUTPUTFILE | cut -d' ' -f 10 | wc -l`
	# 		echo "# Found $nb nodes" > /tmp/$NS2OUTPUTFILE
	# 		cat $NS2OUTPUTFILE >> /tmp/$NS2OUTPUTFILE
	# 		cp /tmp/$NS2OUTPUTFILE $PREFIX$nb.ns2mobility.xml
	# 		echo "==> Found $nb nodes and file is $PREFIX$nb.ns2mobility.xml"
	# 		echo "NS2 File $NS2OUTPUTFILE generated with $NODESESTIMATION !" 
	# 	else
	# 		echo "Error di not find traceExpoter script, are you sure about your SUMO installation ?"
	# 		echo "Did not found traceExporter (py or jar), please review your SUMO installation" 
	# 	fi
    fi
}


# ###############################################################
# usage
usage() { 
    echo "Usage: $0 [-h|--help] [--documentation] --input <OSM File> --nodes <max number of nodes> --sumo <SUMO directory --polytype <polygon type map> [--prefix <file prefix name>] [--sumogui] [--exact <DIR>/SbgMobilityNS.pl]" 1>&2; 
    echo "  input: 
    osm file retreived, please check 
    http://sumo-sim.org/userdoc/Networks/Import/OpenStreetMap.html 
    to retreive an osm map file. 
    Display this script and see some information about sumo installation 
    and osmGet.py script." 1>&2; 
    echo "  nodes: 
    max number of nodes (randomly generated so it's an approximative number)" 1>&2; 
    echo "  sumo: 
    sumo directory (please take care having following scripts and programs: 
    bin/netconvert, bin/polyconvert, tools/trip/randomTrips.py, bin/duarouter, 
    bin/sumo-gui, bin/sumo. 
    The last script should be downloaded from \"somewhere\" 
    frederic/scripts/ReadFCFOutput.pl" 1>&2; 
    echo "  polytype: 
    poly type file (can be found in frederic/scripts/type_sumo_poly.xml). 
    See http://sumo-sim.org/userdoc/Networks/Import/OpenStreetMap.html" 1>&2; 
    echo "  prefix:
    some charaters to prefix file names created (default = sbg)"
    echo "  sumogui:
    display the simulation in sumo-gui"
    echo "  exact
	script (<DIR>/SbgMibilityNS.pl) to limit number of nodes to the number specified in nodes parameters"

    exit 1; 
}

documentation() {
    
    echo "##########################################################################################"
    echo "Sumo configuration to be done to have all tools needed"
    echo "Packages to be installed"
    echo "sudo apt-get install libgdal1-dev proj libxerces-c2-dev"
    echo "sudo apt-get install libfox-1.6-dev libgl1-mesa-dev libglu1-mesa-dev"
    echo "sudo ln -s /usr/lib/libgdal1.7.0.so /usr/lib/libgdal.so"
    echo "SUMO recompilation"

    echo "##########################################################################################"
    echo "For openstreetmap"
    echo "GET http://api.openstreetmap.org/api/0.6/map?bbox=7.7386236,48.5791057,7.756691,48.5865155"
    echo "osmGet.py allow to get large map, for example Strasbourg"
    echo "sumo-0.17.1/tools/import/osm/osmGet.py --prefix sbglarge --bbox \"48.50051, 7.69433, 48.61560, 7.73214\""
    echo "to define the bbox to be used, used geo coordinates, I used http://universimmedia.pagesperso-orange.fr/geo/loc.htm"

    usage
}

# ###############################################################
# Check getOpt version
checkGetOpt() {
    usegetopt=`getopt --test; echo $?`
    if [ $usegetopt -ne 4 ]; then
	echo "getopt problem : you have a version with this problem. See man getopt for more information."
	exit 1 ;
    fi
}

# ##########################################################################################
# Script main
checkGetOpt

# Default Parameters
PREFIX="sbg"
OSMInput=""

# Sumo HOME directory
if [ -z "$SUMOHOME" ]; then
    if [ -n "$SUMO_HOME" ]; then
	SUMOHOME=$SUMO_HOME
    fi
else 
    $SUMOHOME=""
fi


# File needed to add the polygone type
POLYTYPE=""

# Script for the conversion of FCD file (see Sumo documentation) to NS2
# Since I was not able to make working traceExporter.py (see http://sumo-sim.org/userdoc/Tools/TraceExporter.html), I build my own script in perl
#ReadFCFOutput="NONE"
exact="NA"

# end simulation time = number of nodes
NODES=100

# Display simulation in sumo-gui
NoSumoGui=1

# Default options on my machine (frederic's machine)
# if [ -n "$USER" ]; then
#     if [ "$USER" = "frederic" ]; then
# 	echo "Default configuration for frederic's machine"
# 	SUMOHOME=/home/frederic/Sumo/sumo
# 	POLYTYPE=/home/frederic/Development/vanet-ns3/ns-3.15/frederic/scripts/type_sumo_poly.xml
# 	#ReadFCFOutput=/home/frederic/Development/vanet-ns3/ns-3.15/frederic/scripts/ReadFCFOutput.pl
# 	#exact=/home/frederic/Development/vanet-ns3/ns-3.15/frederic/scripts/SbgMobilityNS.pl
#     fi
# fi

# At least one option
if [ $# -eq 0 ]; then
    usage
    exit 1 ;
fi

# Manage options and start conversion
OPTS=$( getopt -n "$0" -o h --long "help,documentation,nosumogui,sumogui,input:,nodes:,prefix:,sumo:,polytype:,script:,exact:"  -- "$@" )
if [ $? != 0 ]
then
    exit 1
fi

eval set -- "$OPTS"

while true ; do
    case "$1" in
	-h|--help) usage; break;;
	--documentation) documentation; break;;
	--input) OSMInput=$2; shift 2;;
	--nodes) NODES=$2; shift 2;;
	--prefix) PREFIX=$2; shift 2;;
	--sumo) SUMOHOME=$2; shift 2;;
	--polytype) POLYTYPE=$2; shift 2;;
	--script) ReadFCFOutput=$2; shift 2;;
	--nosumogui) NoSumoGui=1; shift;;
	--sumogui) NoSumoGui=0; shift;;
	--exact) exact=$2; shift 2;;
	--) mainStart $OSMInput $NODES $PREFIX $SUMOHOME $POLYTYPE $exact $NoSumoGui; shift; break;;
    esac
done

exit 0
