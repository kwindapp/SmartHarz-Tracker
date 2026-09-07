With the device completely powered off:

Use continuity mode between each button terminal and the labelled GND hole.
One side will have continuity to GND—do not use that side.
The other side is the button signal, GPIO42.
Solder WS80 TX to that non-GND button terminal.
Connect WS80 GND to the labelled GND hole.
Downlinks 

T-Echo button functions:

Button/action	Function
Reset button — press once	Restart/power on.        ON Top by Antenna 
Reset button — press twice quickly	Enter bootloader/DFU mode
Program button — press once	Change display page
Program button — press twice	Send an ad-hoc ping
Program button — press 3 times	Enable/disable GPS
Program button — press 4 times	Enable/disable display light
Program button — hold	Turn the T-Echo off
Top capacitive touch button	Refresh/wake the display

To turn it off, hold the Program button for several seconds.

Important for your modified T-Echo: your Program button/GPIO42 is connected to the WS80 serial signal. Disconnect the WS80 signal wire before using that button, because incoming serial data can interfere with button actions. Also, these functions depend on firmware support—your custom firmware may not implement all Meshtastic button actions.









[LoRaWAN] Downlink FPort 1 (HEX): 02.  AQ== Default
[LoRaWAN] New interval: 2 minute(s)    Ag==

[LoRaWAN] Downlink FPort 1 (HEX): 03
[LoRaWAN] New interval: 3 minute(s)


{
  "id":  "KWind_2026",
  "model": "T-ECHO_WS8xx_LoRa",
  "name": "KWind_T-ECHO_GPS",
  "wind_dir_deg": 251,
  "wind_avg_m_s": 1.34,
  "wind_max_m_s": 1.9,
  "wind_min_m_s": null,
  "humidity": 65,
  "battery_V": 3.02,
  "temperature_C": 25.7,
  "rain_mm": 0,
  "light_lux": 0,
  "uv_index": 0,
  "lora_board_mV": 4887,
  "gps_valid": true,
  "latitude": 47.505394,
  "longitude": 8.74721,
  "gps_satellites": 0,
  "gps_hdop": 7,
  "error": ""
}





<img width="3024" height="4032" alt="IMG_7494" src="https://github.com/user-attachments/assets/2ff580ff-6689-45ef-8b84-e66b9c983474" />

<img width="3024" height="4032" alt="IMG_7495" src="https://github.com/user-attachments/assets/e5af0f84-a8b3-4987-87a1-0437e3e215d6" />




