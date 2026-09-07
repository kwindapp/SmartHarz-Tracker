With the device completely powered off:

Use continuity mode between each button terminal and the labelled GND hole.
One side will have continuity to GND—do not use that side.
The other side is the button signal, GPIO42.
Solder WS80 TX to that non-GND button terminal.
Connect WS80 GND to the labelled GND hole.
Downlinks 

T-Echo button functions:

Button/action	Function
Reset button — press once	Restart/power on
Reset button — press twice quickly	Enter bootloader/DFU mode
Program button — press once	Change display page
Program button — press twice	Send an ad-hoc ping
Program button — press 3 times	Enable/disable GPS
Program button — press 4 times	Enable/disable display light
Program button — hold	Turn the T-Echo off
Top capacitive touch button	Refresh/wake the display

To turn it off, hold the Program button for several seconds.

Important for your modified T-Echo: your Program button/GPIO42 is connected to the WS80 serial signal. Disconnect the WS80 signal wire before using that button, because incoming serial data can interfere with button actions. Also, these functions depend on firmware support—your custom firmware may not implement all Meshtastic button actions.









[LoRaWAN] Downlink FPort 1 (HEX): 02.  AQ==
[LoRaWAN] New interval: 2 minute(s)    Ag==

[LoRaWAN] Downlink FPort 1 (HEX): 03
[LoRaWAN] New interval: 3 minute(s)


<img width="3021" height="2864" alt="IMG_7492" src="https://github.com/user-attachments/assets/0e8a2dea-191b-4b74-92fe-1bb8323722c9" />




<img width="885" height="1479" alt="IMG_7489" src="https://github.com/user-attachments/assets/4e67a758-a283-4410-9102-58542f6d2ea8" />


