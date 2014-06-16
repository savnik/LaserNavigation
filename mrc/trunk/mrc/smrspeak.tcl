#!/usr/bin/tclsh

# Configuration parameters.
# (Easy)
#

set sound(joystick.file)  R2.wav

set sound(normal.files)   [list norm1.wav norm2.wav]
set sound(normal.delay)    5000
set sound(normal.repeat)  10000

set sound(stopped.files)  [list flyt1.wav flyt2.wav flyt3.wav]
set sound(stopped.delays) [list 4000     3000     2000     ]

# Configuration procedures.
# (Tricky)
#

proc joystick {} {
    global sound
    play $sound(joystick.file)
}

proc normal {} {
    global sound
    do_after $sound(normal.delay) normal_repeat
}

proc normal_repeat {} {
    global sound
    play [next_rotate sound(normal.files)]
    do_after $sound(normal.repeat) normal_repeat
}

proc stopped {} {
    global sound
    stopped_repeat $sound(stopped.files) $sound(stopped.delays)
}

proc norm {} {
    global sound
    play "norm1.wav"
}

proc obstacle {} {
    global sound
    play "flyt1.wav"
}

proc obstacle1 {} {
    global sound
    play "flyt2.wav"
}
proc stopped_repeat {files delays} {
    set file  [lindex $files  0]
    set delay [lindex $delays 0]
    if {[llength $files]  > 1} {set files  [lrange $files  1 end]}
    if {[llength $delays] > 1} {set delays [lrange $delays 1 end]}
    play $file
    do_after $delay stopped_repeat $files $delays
}

##############################################
# No user servicable parts after this comment.
# Enter at your own risk.

proc play file {
    puts "play $file"
    if {[catch {exec play smrsound/$file} error]} {
	puts $error
    }
}

proc next_rotate l {
    upvar $l x
    set next [lindex $x 0]
    set x [lrange $x 1 end]
    lappend x $next
    return $next
}

proc do_after {delay args} {
    global sound
    set sound(after_id) [after $delay $args]
}

proc stop_current {} {
    global sound
    after cancel $sound(after_id)
}

#set sound(s) [socket localhost 24900]
set sound(s) stdin
fconfigure $sound(s) -buffering line -translation {auto lf}
set sound(after_id) dummy

puts $sound(s)
while { 3 >= 2 } {
    after cancel $sound(after_id)
    
    if {[eof $sound(s)]} {exit 0}
    if {[catch [exec /usr/lib/ViaVoiceTTS/samples/cmdlinespeak/cmdlinespeak [gets $sound(s)]] error]} {puts $error}
}
#    if {[catch [gets $sound(s)] error]} {puts $error}
#    if {[catch {puts $sound(s) ""}]} {exit 0}
#}

#fileevent $sound(s) readable read_socket
#puts $sound(s) ""			;# start handshake
