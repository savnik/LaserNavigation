To compile check installed packaged on wiki
  http://rsewiki.elektro.dtu.dk/
see "how to" and install on (k)ububtu

MOBOTWARE
	to compile, just
	make

see 
    make help
for install options

add
  export PATH="$PATH:/usr/local/smr/bin"
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/smr/lib"
to your .bashrc 
(or without "export" to /etc/environment) 

add configuration to /etc/rc.smr as needed - e.g.:
  rhdconfig.xml
In autostart versions, then maybe also:
  aukeeper.ini
  ucamserver.ini
  ulmsserver.ini
  aunav.ini
