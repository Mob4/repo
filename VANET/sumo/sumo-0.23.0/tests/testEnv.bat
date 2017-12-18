set TEXTTEST_HOME=%CD%
IF "%SUMO_HOME%"=="" SET SUMO_HOME=%CD%\..
set ACTIVITYGEN_BINARY=%CD%\..\bin\activitygen%1.exe
set DFROUTER_BINARY=%CD%\..\bin\dfrouter%1.exe
set DUAROUTER_BINARY=%CD%\..\bin\duarouter%1.exe
set JTRROUTER_BINARY=%CD%\..\bin\jtrrouter%1.exe
set NETCONVERT_BINARY=%CD%\..\bin\netconvert%1.exe
set NETGENERATE_BINARY=%CD%\..\bin\netgenerate%1.exe
set OD2TRIPS_BINARY=%CD%\..\bin\od2trips%1.exe
set SUMO_BINARY=%CD%\..\bin\sumo%1.exe
set POLYCONVERT_BINARY=%CD%\..\bin\polyconvert%1.exe
set GUISIM_BINARY=%CD%\..\bin\sumo-gui%1.exe
set MAROUTER_BINARY=%CD%\..\bin\marouter%1.exe
set EMISSIONSDRIVINGCYCLE_BINARY=%CD%\..\bin\emissionsDrivingCycle%1.exe
set EMISSIONSMAP_BINARY=%CD%\..\bin\emissionsMap%1.exe

SET TEXTTESTPY=texttest.py
python -c "import texttestlib"
IF NOT ERRORLEVEL 1 SET TEXTTESTPY=texttest.pyw
