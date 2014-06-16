log "$xkalman" "$ykalman" "$thkalman" "$gpseasting" "$gpsnorthing"

d1 = 10
n=4
drive @v2.0 : ($drivendist >4.5)
fwd 0.5
set "usekalmanodo" 1
targethere
label "a"

drive @v2.0 : ($drivendist >d1)
turnr 3 90
fwd 0.5
fwd -3.5
turnr 3 90
drive @v2.0 : ($drivendist >d1)

turnr 3 -90
fwd 0.5
fwd -3.5
turnr 3 -90
n=n-1
eval $odox; $odoy; $odoth

if(n>0) "a"


stop

