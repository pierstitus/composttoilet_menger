# Aansturing automatische composttoilet

## Handleiding

- Verbinden met wifi 'Composttoilet', wachtwoord als opgegeven in localconfig.h.
- ga naar http://192.168.4.1/, daar kan je de status zien en configuratie aanpassen.
- onderin kan je de wifi instellingen aanpassen zodat hij met een wifi router verbindt. Dat is nodig om de huidige tijd te weten. Als dat werkt kan je ook via http://composttoilet.local verbinding maken via dat wifi, of als dat niet werkt via het adres wat bovenaan de pagina wordt weergegeven.

## Firmware updaten

Ga naar composttoilet.local/update en upload het nieuwe firmware bestand (firmware.bin)

## Ontwikkelen

### Specificaties

#### Mengprogramma:
Voor een aantal klassen weerstand:
Bij draaiweerstand w0-w1:
1. y seconden linksom draaien met snelheid z
2. x seconden wachten 3
3. y1 seconden rechtsom draaien met snelheid z1
4. x1 seconden wachten

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
- Luchtpomp aan/uit

Pinout:
- Temperatuursensor op D3 (pull-up)
- Rotatiesensor op i2c
- Bril open/dicht sensor op D8 (pull-down)

### Software

De software is ontwikkeld met PlatformIO, wat als plugin in VSCode geladen kan worden. Dat installeert zelf de benodigde pakketten en geeft bij "Project Tasks->d1_mini->General" de optie "Build". Dat maakt het bestand firmware.bin dat via de web interface kan worden geupload.

### Onderdelen

- ESP8266 microcontroller: WeMOS D1 R2
- Gear motor: Takanawa 555
- Motorcontroller: Double BTS7960 43A H-bridge High-power Motor Driver
- Temperatuursensor: waterproof DS18B20
- Elektronisch kompas (voor rotatie mengas meten): QMC5883L, meet de orientatie van een klein magneetje dat op de as is gelijmd
- 2x Relay: Relay module 2 relays, 5V
- Magneetsensor (bril open/dicht): MC38
- Emergency switch: Push Button Switch 1 NO 1 NC 10A 660V Emergency
- Stroomsensor (optioneel): Hall Current Sensor Module ACS712 20A model for arduino
