AT+MIPLCREATE
AT+MIPLADDOBJ=0,3320,1,"1",1,0
AT+MIPLADDOBJ=0,3336,1,"1",2,0
AT+MIPLOPEN=0,86400
AT+MIPLOBSERVERSP=0,00000,1
AT+MIPLOBSERVERSP=0,00000,1
AT+MIPLDISCOVERRSP=0,00000,1,4,"5700"
AT+MIPLDISCOVERRSP=0,00000,1,9,"5513;5514"
AT+MIPLNOTIFY=0,000000,3320,0,5700,4,4,100,0,0
AT+MIPLNOTIFY=0,000000,3336,0,5513,1,8,"00.00000",1,0
AT+MIPLNOTIFY=0,000000,3336,0,5514,1,9,"000.00000",0,0

=================================================================

AT+MIPLCREATE

AT+MIPLADDOBJ=0,3311,1,"1",4,2

AT+MIPLOPEN=0,86400

AT+MIPLOBSERVERSP=0,78671,1

AT+MIPLDISCOVERRSP=0,,1,19,"5850;5851;5706;5805"


AT+MIPLNOTIFY=0,78671,3311,0,



AT+QGNSSC=1
AT+QGNSSC?
AT+QGNSSRD="NMEA/RMC"

$GNRMC,075119.00,A,3027.7399,N,11436.8805,E,0.618,,280622,,,A,V*1A
$GNRMC,083427.00,A,3027.5416,N,11436.9428,E,1.122,,280622,,,A,V*14

经度：11436.9428		114.615713333333333
纬度：3027.5416		30.459026666666667