

Specificaties:
Mengprogramma:
Voor een aantal klassen weerstand:
Bij draaiweerstand w0-w1: y seconden linksom draaien met snelheid z - x seconden wachten - y1 seconden rechtsom draaien met snelheid z1 - x1 seconden wachten

Bij niet draaien: proberen hard heen en weer te draaien, anders stoppen/pauze

Als bril open niet draaien.
Als bril 24 uur niet geopend is -> vakantiemodus

Weerstand wordt niet direct gemeten, maar is af te leiden uit draaisnelheid en motorvoltage, bijv. uitgedrukt in rpm/V

Alle sensordata uploaden naar een cloudservice, elke t seconden.
Opties:
- Intern geheugen (SPIFFS): csv file, max 3mb
- Google sheets: http://embedded-lab.com/blog/post-data-google-sheets-using-esp8266/


Parameters instellen via Wifi.

Sensoren:
- Rotatie mengas
- Temperatuur compost
- Bril open/dicht sensor
- Evt. stroomsterkte door motor (niet noodzakelijk, maar misschien interessant?)
- opties later: Infrarood/lichtreflectie compost, vochtmeter. I2C en analoge pin beschikbaar.

Uitvoer:
- Draairichting en snelheid motor
- Verwarmingselement aan/uit

Pinout:
- Temperatuursensor op D3 (pull-up)
- Rotatiesensor op i2c
- Bril open/dicht sensor op D8 (pull-down)


# Handleiding

Ga naar composttoilet.local

Data wordt intern of extern opgeslagen, daar kan ook een grafiek bekeken worden.


## Firmware updaten
Ga naar composttoilet.local/update en upload het nieuwe firmware bestand (firmware.bin)
