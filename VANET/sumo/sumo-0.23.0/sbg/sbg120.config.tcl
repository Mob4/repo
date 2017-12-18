# set number of nodes
set opt(nn) 92

# set activity file
set opt(af) $opt(config-path)
append opt(af) /sbg120.activity.tcl

# set mobility file
set opt(mf) $opt(config-path)
append opt(mf) /sbg120.ns2mobility.xml

# set start/stop time
set opt(start) 0.0
set opt(stop) 120.0

# set floor size
set opt(x) 2789.08
set opt(y) 3128.95
set opt(min-x) 1058.02
set opt(min-y) 1417.18

