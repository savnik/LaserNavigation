gyrooffset = 12.340615
gyrogain = -4.84814e-06
set "gyrotype" 1
routeinput "mrc" "odoconnect" "dth" "$fogPhdZ"
set "$gyro1gain" gyrogain
set "$gyro1off" gyrooffset
set "odocontrol" 1
control "removelogvars"
log "$time" "$odox" "$odoy" "$odoth" "$gpseasting" "$gpsnorthing"
log "$xkalman" "$ykalman" "$thkalman" "$hakosteeringangleref" "$hakosteeringangle" "$hakospeedref"
log "$kalmanmode" "$kalmanstatus" "$gpsquality" "$gpsdop" "$gpssatellites" "$odovelocity"
control "startlog"
set "kalmanon" 1
set "hakoliftinggearstateref" 130
set "usekalmanodo" 1
fwd 10 @v0.4
set "hakoenginespeed" 1900
driveon 707884.99 6174119.18 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707871.89 6174118.25 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707867.12 6174114.90 -85.9 @v0.7 : ($targetdist < 1.0)
driveon 707872.32 6174112.26 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707885.42 6174113.20 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707890.69 6174109.56 -85.9 @v0.7 : ($targetdist < 1.0)
driveon 707885.98 6174105.22 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707872.89 6174104.28 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707868.11 6174100.94 -85.9 @v0.7 : ($targetdist < 1.0)
driveon 707873.31 6174098.30 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707886.41 6174099.23 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707891.11 6174103.58 94.1 @v0.7 : ($targetdist < 1.0)
driveon 707885.84 6174107.21 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707872.74 6174106.28 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707867.97 6174102.93 -85.9 @v0.7 : ($targetdist < 1.0)
driveon 707873.17 6174100.29 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707886.27 6174101.23 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707890.76 6174108.56 94.1 @v0.7 : ($targetdist < 1.0)
driveon 707885.27 6174115.19 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707872.18 6174114.26 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707867.40 6174110.91 -85.9 @v0.7 : ($targetdist < 1.0)
driveon 707872.60 6174108.27 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707885.70 6174109.21 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707890.90 6174106.57 -85.9 @v0.7 : ($targetdist < 1.0)
driveon 707886.13 6174103.22 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707873.03 6174102.29 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707867.76 6174105.92 94.1 @v0.7 : ($targetdist < 1.0)
driveon 707872.46 6174110.27 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707885.56 6174111.20 4.1 @v0.7 : ($targetdist < 1.0)
driveon 707890.33 6174114.55 94.1 @v0.7 : ($targetdist < 1.0)
driveon 707885.13 6174117.19 -175.9 @v0.7 : ($targetdist < 1.0)
driveon 707872.03 6174116.25 -175.9 @v0.7 : ($targetdist < 1.0)
set "hakoenginespeed" 800
fwd 1 @v0.2 : ($targetdist < 0.0) | ($odovelocity < 0.3)
idle
control "savelog" "mrc_south11.logg"
