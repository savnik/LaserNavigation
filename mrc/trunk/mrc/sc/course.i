% course s03
log "$time" "$irdistleft" "$irdistfrontleft" "$irdistfrontmiddle" "$irdistfrontright" "$irdistright"
n=0
thend=0
%log "thend"
label "start"
set "$odox" 0
set "$odoy" 0
set "$odoth" 0
%
%goto "dock"
% find box
%goto "box"
%goto "wall"
%goto "findgate"
ignoreobstacles
followline "br" @v0.25 :($drivendist > 1.3)
ignoreobstacles
%followline "br" @v0.25 :($odoy <-1.95 )
%fwd 0.05
%idle
%eval -$odoy; $odox ;$odoth/3.1416*180
%goto "endcourse"

ignoreobstacles
followline "br" :($irdistfrontmiddle <0.25)
ignoreobstacles
fwd 0.05
idle
wait 2
d=$irdistfrontleft+$irdistfrontmiddle+$irdistfrontright
eval -$odoy+0.23+d/3 
%d1=-$odoy
%x=-$odox
%turn 180
%fwd d1
%turn -90
%fwd -0.4
%goto "endcourse"
% go to front of next obstacle
l=1.13
tmp=-$odoy-2
th=atan2(tmp,l)*180/3.1415
d=sqrt(tmp^2+l^2)
th1=90+th
turn th1
fwd d
th1=-90-th
turn th1
label "box"
% move box
ignoreobstacles
followline "bm" :($drivendist > 0.3 & $crossingblackline)
ignoreobstacles
fwd 0.13
fwd -1.1
turn -90

% drive through box gates

%drive @v0.2 :($drivendist>0.2 & $blacklinefound)
%turnr  0.21 90
fwd 0.55
turn 90
drive @v0.2 :($drivendist>0.2 & $blacklinefound)
fwd 0.21
turn 90
followline "bm"  @v0.2: ($drivendist >0.2 & $crossingblackline)
%followline "bl" @v0.2 :($drivendist > 1)
%goto "dock"
drive :($drivendist > 0.3)
% goto wall
label "findgate"
followline "br": ($crossingblackline & $drivendist > 0.8)
label "nogate"
followline "bm" @v0.05: ($irdistleft < 0.35)
stop
wait 0.3
if ($irdistleft > 0.35) "nogate"
wait 0.3
if ($irdistleft > 0.35) "nogate"
followline "bm" @v0.3: ($drivendist >0.37)
fwd 0.05
turn 90
fwd 0.9 
turn -90
ignoreobstacles
drive @v0.25 :($drivendist >0.3 & ($blacklinefound | $irdistfrontmiddle < 0.30))
%turnr 0.15 90
ignoreobstacles
fwd 0.21
turn 90
label "wall"
followline "bm" :($drivendist > 0.4 & $crossingblackline)
%turnr 0.21 90
 fwd 0.21
 turn 90
% follow wall

%followwall "l" 0.25 :($drivendist > 0.9 )
drive @v0.2 :($drivendist > 1.2 & $irdistleft > 0.5)
fwd 0.39
turn 90
fwd 0.9
turn 90
drive :($drivendist > 1 & $blacklinefound)
ignoreobstacles
fwd 0.21
turn 90


%drive :($drivendist > 0.03)
%turnr  0.32 90
%drive  @v 0.3:($drivendist > 0.4)
%turnr 0.25 93
%drive :($drivendist > 1 & $blacklinefound)
%turnr 0.21 90
%goto "wall"
%turnr 0.25 -90
%drive :($drivendist > 0.4 & $blacklinefound)
%turnr 0.21 -90

% go to precision gate

followline "bm"  @v0.3:(  $crossingblackline)
fwd 0.21
turn 45 @v0.2
drive :($drivendist > 0.9)

% follow white line
followline "wm" :($drivendist > 1.8 & $blacklinefound)
% To goal
fwd 0.21
turn -90

% open door
label "dock"
ignoreobstacles
followline "bm" @v0.2 :($irdistfrontmiddle <0.25)
ignoreobstacles
fwd 0.05
turn 90 @v0.3
fwd 0.65
turn -90
ignoreobstacles
fwd 0.70
turn 90
fwd -0.12
turn -90
fwd -0.8 @v0.8 @a0.6
turn -90 @v0.2 @a0.6
thend=$odoth
ignoreobstacles
drive @v0.2 :($blacklinefound)
ignoreobstacles
fwd 0.21
turn  90
ignoreobstacles
followline "bm" @v0.3 :($drivendist > 0.2 & $irdistfrontmiddle <0.2)
ignoreobstacles
idle
n=n+1
eval n
goto "endcourse"
fwd -0.8
turn 180
followline "bm" @v0.3 :($crossingblackline)
fwd 0.21
turn -90
drive :($drivendist > 1.5)
drive :($blacklinefound)
fwd 0.21
turn -90
followline "bm" @v0.3 :($crossingblackline)
fwd 0.21
turn -90
fwd -0.3
goto "start"
label "endcourse"
idle
