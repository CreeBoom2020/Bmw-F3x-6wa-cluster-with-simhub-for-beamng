Hello This is a scetch for the bmw f3x series cluster using simhub and beamng.

What i used
an arduino uno and a seeed studio can bus shield clone

pinout

1,2,11 12v

7,8 gnd

6 CAN H

12 CAN L

4,5 ntc temp sensor maybe 10k? (optionally)

for cruise control doors trunk frunk read out u need to replace the simhubextras.lua file to this directory SteamLibrary\steamapps\common\BeamNG.drive\lua\vehicle\extensions\auto with the one thats here.

if u dont want to do that chnage the EDITEDSELUA to false!

from standart the scetch runs so that it send the RPM for a gasoline RPM disk on a diesel cluster since mine broke and changed the RPM disks from a nother cluster.
to fix that just change GRPMOND to false in the .ino file!

curent working thinks;
RPM

Speed

How full the tank is

engine oil temp

EfficientDynamics

consumption(a bit bugy sometimes workes sometimes not :/)

trip km or miles

engine water overhated mesage

if a tire is deflated displayed on cluster if it is and wich one it is

door open message for each door in game (requires simhubextras.lua patch)

trunk open message (requires simhubextras.lua patch)

hood open message (requires simhubextras.lua patch)

drive mode select ( manuali in .ino file)

change the language and other cluster seting(look in .ino file)

cruisecontrol(requires simhubextras.lua patch)(only lamp working on speed readout)

tsc lamp

high beam lamp

low beam lamp

fogg light lamp

backlight

check engine light

gear indecator (til 9)


lots of thanks to 

@r00li for his carcluster project! since i took a few ids from that

@VintageCollector for his g20 cluster demo test for ets2! also took a few ids from him

@mr_goofy for help with the cruise control!
