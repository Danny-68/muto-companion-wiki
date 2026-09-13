# Muto Companion — Status & Roadmap (briefing voor Claude Code)

Datum: 11 september 2026
Doel van dit document: een volledig zelfstandig startpunt, zodat een nieuwe sessie (Claude Code) kan beginnen met bouwen zonder de hele geschiedenis opnieuw te hoeven doorlopen.

## Sessie afgesloten 11 sep 2026, ~21:15 — meteen lezen bij een nieuwe sessie

Gebruiker gaf expliciet aan te stoppen en morgen verder te gaan. Robot stond bij het afsluiten stil en veilig: `app_muto.py` draait normaal en is enige eigenaar van `/dev/myserial`, `mutod` draait NIET, geen enkel ROS2-stackproces (LiDAR/EKF/rf2o/driver/joy) actief in `humble_run`. Geverifieerd met `fuser`/`pgrep`/`docker exec ps` vlak voor het afsluiten.

**Laatste voorgestelde actie, NIET uitgevoerd:** `cd /home/pi && DRIVER=phoenix_driver bash muto_fase1_start.sh` — dit was klaargezet (STAP 0b voor mutod is af en los getest, zie Fase 2b hieronder) en de gebruiker koos ervoor het zelf handmatig te draaien i.p.v. via Claude Code (de auto-mode-classifier blokkeerde het commando twee keer, ook na expliciete bevestiging in de chat — dit is een harness-niveau-blokkade, geen omissie van de gebruiker). Onbekend of de gebruiker dit voor het einde van de sessie alsnog zelf gedraaid heeft — controleer bij een nieuwe sessie eerst de live status (poort-eigenaar, of `mutod`/de ROS2-stack draait) voor je aanneemt dat de stack nog (niet) staat, in plaats van op dit document te vertrouwen voor actuele runtime-status.

**Als de stack alsnog gestart is:** volg de "wat te controleren"-lijst die aan de gebruiker gegeven is (zie sectie hieronder, of vraag opnieuw) — met name `/tmp/mutod.log` en `docker exec humble_run cat /tmp/phoenix_driver.log` bij problemen.

---

## 0. Project in één alinea

Muto RS (Yahboom hexapod, Pi 5 + STM32F103RCT6-baseboard + 18× CSPower bus-servo's) krijgt een companion-laag geïnspireerd op — niet gekopieerd van — Pollen Robotics' Microduck-architectuur (open-source, Rust, `duck-control`/`robotd`/`duck-ipc-proto`). Doel: eigen gedrag (rondlopen, interactie, gebaren) plus latere communicatie/coördinatie met een fysieke Microduck (levering ~december 2026). Fase 1 (hardware-protocol volledig doorgronden) is per vandaag afgerond.

---

## 1. Fase 1 — Resultaat: het complete, geverifieerde protocol

Alles hieronder is **getest op de echte robot**, niet afgeleid uit documentatie alleen.

### 1.1 Frame-formaat (STM32-baseboard, `/dev/myserial`, 115200 baud, 8N1)

```
Commando (Pi → STM32):  0x55 0x00 | lengte | 0x01(write)/0x02(read) | adres | data... | checksum | 0x00 0xAA
Antwoord  (STM32 → Pi):  0x55 0x00 | lengte | 0x12(data-return)      | adres | data... | checksum | 0x00 0xAA

lengte    = (aantal databytes) + 8
checksum  = 255 - ((lengte + mode + adres + som(databytes)) % 256)
```

Python-referentie-implementatie (gevalideerd, hergebruik dit):
```python
def _cs(*b):
    return (0xFF - sum(b)) & 0xFF

def send_cmd(ser, addr, data_bytes):
    length = len(data_bytes) + 0x08
    mode = 0x01  # write
    value_sum = sum(data_bytes)
    checksum = (255 - ((length + mode + addr + value_sum) % 256)) & 0xFF
    tx = [0x55, 0x00, length, mode] + [addr] + data_bytes + [checksum, 0x00, 0xAA]
    ser.write(bytes(tx))
```

### 1.2 Adrestabel (definitief)

| Adres | Naam (MutoLibCore) | Richting | Functie | Status |
|---|---|---|---|---|
| 0x06 | RESET | write | Standing posture / reset | Gebruikt, werkt |
| 0x11 | STAY_PUT | write | Stopt lopende gait, benen zetten neer | In MutoLibCore, niet eerder in deze briefing vermeld — nu gebruikt door mutod's deadman |
| 0x12-0x17 | FORWARD/BACKWARD/SHIFT_L/R/TURN_L/R | write | Gait-commando's | Stabiel, in productie |
| 0x26 | TORQUE_ON | write | Torque aan (servo_id, 0=alle) | Gevalideerd |
| 0x27 | TORQUE_OFF | write | Torque uit (servo_id, 0=alle) | Gevalideerd |
| **0x40** | MOTOR | write | **Eén servo naar hoek, `[servo_id, angle&0xFF, runtime_hi, runtime_lo]`** | **Veilig, 10/10 stress-test zonder fouten. LET OP: runtime is hi-byte-eerst op de draad (geverifieerd tegen MutoLibCore.motor(), zie §1.2a) — wijkt af van een eerdere, foutieve lo/hi-aanname in deze briefing.** |
| 0x41 | LEG | write | Drie servo's van één poot tegelijk | **⚠️ VERMIJDEN — veroorzaakte ooit `OSError: No such device`. Gebruik 3× 0x40 in plaats hiervan, functioneel gelijkwaardig, getest veilig.** |
| **0x50** | MOTOR_ANGLE | read | **Bulk-read: ALLE 18 servo-hoeken in één transactie, 18 databytes** | **Bevestigd, ~27ms round-trip** |
| 0x60 | ATTITUDE_ANGLE | read | **IMU (roll/pitch/yaw), NIET servo-gerelateerd — negeert ID-parameter** | Bevestigd; dataframe (8 databytes) nog niet ontcijferd |
| 0x61 | IMU_RAW | read | Ruwe 9-axis IMU data | Niet getest |
| 0x3E | ACTION | write | 9 performance-actiegroepen (reset/stretch/greet/wave/spin/...) | Niet getest — **kandidaat voor de "wave"-skill, test dit eerst voor je iets nieuws bouwt** |

**1.2a — runtime-byte-volgorde, gecorrigeerd 11 sep 2026:** bij het bouwen van `mutod/protocol.py` is de MOTOR-write geverifieerd tegen de daadwerkelijke `MutoLibCore.motor()`-broncode (humble_run container). Die stuurt `struct.pack('<h', runtime)` en zet vervolgens `value[1]` (hoge byte) vóór `value[0]` (lage byte) op de draad — dus **hi, lo**, niet lo, hi zoals een eerdere versie van deze briefing suggereerde. De library-code is de bron van waarheid, niet de eerdere prozabeschrijving.

### 1.3 Servo-ID-mapping en eenheid

- ID 1-18, drie per poot: coxa/femur/tibia.
- Poot-volgorde: RF=[1,2,3], RM=[4,5,6], RR=[7,8,9], LR=[10,11,12], LM=[13,14,15], LF=[16,17,18].
- Bus-architectuur: USART2 (rechterpoten, ID 1-9), USART3 (linkerpoten, ID 10-18) — beide via de STM32, niet direct door de Pi aan te spreken.
- **Hoek-eenheid: signed int8, graden** (`byte - 256` als byte > 127). Gevalideerd via commando→lezen-vergelijking, <4° afwijking.
- **0x50's 18 databytes = servo 1 t/m 18 op volgorde**, elk 1 byte, signed.

### 1.4 Timing (gemeten, niet geschat)

| Operatie | Tijd |
|---|---|
| 1× `read_motor()` (0x50, alle 18 servo's) | 27,3ms gemiddeld (25-28ms bereik) |
| 1× `motor()` write (0x40) | 0,02ms (transmissie; werkelijke bewegingstijd via `runtime_ms`-parameter) |
| **Volledige control-tick (1 read + N writes)** | **~27-28ms, gedomineerd door de read** |

**Conclusie: richt een control loop in op ~20-25Hz (40-50ms cyclustijd), niet op Microduck's 50Hz (20ms).** Dit is een harde hardwarelimiet van de STM32-bulk-read, niet van de Pi.

### 1.5 Al bestaande, werkende code (niet opnieuw bouwen)

- `/home/pi/foot_contact.py` — contact-detectie. **Gebruikt nu nog 0x60 (fout, was IMU niet servo) — moet herbouwd worden op 0x50.** Retry-logica en threading-structuur zijn wel herbruikbaar.
- `phoenix_gait.py` — eigen tripod/ripple/wave/centipede gait-engine, gebruikt runtime-register (0x2C/2D) voor 18ms hardware-interpolatie. Werkt, yaw-drift ruim beter dan firmware-gait.
- `body_pose.py` — breathe/sweep/circle/periscope met IK-degradatie.
- `muto_driver_fixed.py` — huidige ROS2/cmd_vel-driver, gebruikt alleen simpele gait-commando's (0x12-0x17).
- `muto_driver_v2.py` — yaw-drift-fix via odometrie, **klaar maar nog niet gedeployed.** Losse, onafhankelijke taak. (Niet gevonden op de Pi als los bestand tijdens Fase 2-zoektocht 11 sep 2026 — mogelijk alleen als diff/patch bewaard, of onder een andere naam. Controleer bij gebruik.)
- `app_muto.py` — Yahboom's eigen app. **Moet permanent uit de opstartketen zodra `mutod.py` bestaat** (voorkomt poort-conflicten, is al eerder een probleem geweest). **Stond op 11 sep 2026 nog actief te draaien (PID 2529, autostart via `~/.config/autostart/app.desktop`) — nog NIET uitgeschakeld, zie voortgang Fase 2 hieronder.**
- `MutoLibCore.py` (in Docker, `/usr/local/lib/python3.10/dist-packages/muto_hexapod_lib/core/`) — bevat `motor()`, `read_motor()`, `Servo_torque_on/off()`, `reset()`, gait-methodes. Broncode is leesbaar, geen reverse engineering nodig.

---

## 2. Architectuurbeslissingen (vastgelegd, niet heropenen zonder reden)

1. **`mutod.py` wordt de enige schrijver naar `/dev/myserial`.** `muto_driver_fixed.py` stuurt zijn `cmd_vel` door naar `mutod` i.p.v. zelf te schrijven. `app_muto.py` gaat uit de opstartketen.
2. **IPC-schema volgt `duck-ipc-proto`** (Rust, Pollen Robotics), maar geïmplementeerd in Python — geen Rust-dependency nodig voor deze laag. Veldnamen: `robot.move` = `{vx, vy, vyaw}`, skills via `robot.do`.
3. **Discovery via UDP-broadcast**, niet BLE — Muto en een toekomstige Microduck delen een LAN. Coördinatiepatroon (settle/stale/deterministisch leiderschap) van chorale overnemen, transport aanpassen.
4. **Camera is ondergeschikt aan LiDAR/odometrie** voor beweging — camera alleen voor gericht kijken tijdens interactie en (later) bearing-naar-Microduck.
5. **Control-looptempo: ~20-25Hz**, niet 50Hz (zie §1.4).
6. Taalkeuze voor `mutod.py`: **Python.** MutoLib is Python, het protocol is nu volledig eigen (geen Rust-crate-hergebruik nodig zoals aanvankelijk overwogen bij Microduck-vergelijking), en de control-rate (20-25Hz) heeft geen Rust-prestaties nodig.
7. **SLAM/AMCL-gebaseerde absolute lokalisatie: GEPARKEERD (12 sep 2026, herbevestigd na opnieuw dezelfde convergentiefout).** Geschiedenis: RTAB-Map (camera-SLAM) al in juli 2026 afgeschreven wegens Pi/Jetson/WiFi-belasting ([[muto_rtabmap_abandoned_for_load]]); AMCL bleek in augustus 2026 structureel vast te lopen op symmetrische ruimte-geometrie, geen tuning-probleem ([[muto_localization_approach_pivot_2026-08-21]]), waarna een AprilTag-aanpak gekozen maar nooit gebouwd werd. Op 12 sep 2026 opnieuw exact dezelfde convergentiefout gezien (yaw-covariantie ~5,5 rad², grote raycast-afwijkingen) ondanks een schone spin-kalibratie-fix — gebruiker koos ervoor dit (opnieuw) te laten rusten i.p.v. verder te debuggen. **Nieuwe standaardrichting:** companion-navigatie (Rondlopen/obstakelvermijding) leunt op lokale, kaart-vrije waarneming — LiDAR-vrije-ruimte + odometrie-relatieve novelty-grid (Fase 5, al zo gebouwd, vereist al geen AMCL) — aangevuld met periodieke camera/dieptecamera-checks (Fase 6) i.p.v. een globale kaartpositie. Nav2/AMCL-infrastructuur blijft aanwezig en bruikbaar (bv. voor een toekomstige precieze-waypoint-behoefte), maar is niet langer de standaardweg voor het dagelijkse companion-gedrag.

---

## 3. Roadmap — volgende fasen

### Fase 2 — `mutod.py`: de kern-daemon (eerstvolgende stap)

**Status 11 sep 2026, eind van sessie: code geschreven, offline geverifieerd, ÉN 10 minuten stabiel getest tegen de echte robot. Twee echte bugs gevonden en gefixed die alleen op hardware zichtbaar werden — zie hieronder.**

**Deliverables (locatie: `/home/pi/mutod/`):**
- `mutod/protocol.py` — frame-opbouw, checksum, adrestabel als constanten. **Geschreven en unit-getest tegen `MutoLibCore.py`'s werkelijke bytes** (write-frame voor MOTOR, read-frame voor MOTOR_ANGLE, response-parsing incl. checksum-afwijzing) — allemaal bit-voor-bit identiek aan de referentie-implementatie.
- `mutod/hal.py` — `MutoHAL`-klasse: `read_all_angles() -> dict[servo_id, degrees]`, `write_servo(id, angle, runtime_ms)`, `write_leg(leg_name, [a1,a2,a3], runtime_ms)` (intern als 3× `write_servo`, NOOIT 0x41), `torque(on, servo_id=0)`, `reset_posture()`, `stay_put()`, `gait(direction, step)`.
- `mutod/safety.py` — `Deadman`-klasse (timeout 0.6s, `trip_once()` voorkomt herhaald afvuren), NaN/inf/bereik-validatie op hoeken/servo_id/runtime vóór ze naar `write_servo` gaan.
- `mutod/daemon.py` — hoofdproces: control-loop op 25Hz (40ms tick, valt terug naar 2Hz tijdens actieve beweging) die continu `read_all_angles()` aanroept, deadman-check per tick, TCP-socket (127.0.0.1:8420 default -- **niet** een Unix-socket, zie Fase 2b hieronder voor waarom) met newline-delimited JSON commando's (`get_state`, `gait`, `write_servo`, `write_leg`, `raw_frames`, `read_attitude`, `torque`, `reset_posture`, `stop`).

**Poort-conflictdetectie (acceptatiecriterium "fail loud"): geïmplementeerd én bevestigd werkend.** `_other_holders()` gebruikt `fuser` op het device pad, vóórdat de seriële poort zelf geopend wordt. Getest terwijl `app_muto.py` (destijds PID 2529) daadwerkelijk `/dev/ttyUSB0` open had: `mutod.daemon` weigerde meteen te starten met een duidelijke foutmelding, zonder de poort zelf aan te raken.

**Echte "exclusief" open toegevoegd (TIOCEXCL), niet alleen een startup-check.** Tijdens het testen bleek er een tweede, niet eerder in deze briefing genoemde port-gebruiker te zijn: `yahboom_oled.py` (het OLED-statusschermpje, eigen autostart-entry `~/.config/autostart/oled.desktop`) leest `/tmp/battery_pct`/`/tmp/battery_volt` als primaire bron, en valt alleen terug op een eigen directe `/dev/myserial`-verbinding als die cache-bestanden ontbreken. Zodra `app_muto.py` (de enige schrijver van die cache-bestanden) gestopt werd, viel `yahboom_oled.py` meteen terug op zijn eigen seriële fallback en botste met `mutod`. Opgelost door na het openen van de poort `fcntl.ioctl(fd, termios.TIOCEXCL)` te zetten — een latere open-poging van een ander proces krijgt dan een schone `EBUSY` in plaats van dat er frames door elkaar gaan lopen. Bevestigd: een tweede `serial.Serial()`-poging tijdens een lopende `mutod` faalt netjes met `Device or resource busy`.

**Gevonden en gefixte bug in `protocol.py`:** de eerste implementatie van `parse_response()` verwachtte `length == expect_count + 9` voor het LEN-veld van een read-antwoord. Dat is fout — het moet `expect_count + 8` zijn (dezelfde conventie als bij write-frames). Alle offline unit-tests waren zelf-consistent gebouwd met dezelfde foute aanname en gaven dus vals-positief "OK". Pas de eerste echte hardware-read (na het draineren van stale bytes) legde het off-by-one bloot: de STM32 antwoordde correct, maar `parse_response` verwierp het antwoord. Gefixed; nadien 15/15 en vervolgens 10 minuten lang 100% geslaagde reads.

**10-minuten-stabiliteitstest: geslaagd.** `app_muto.py` tijdelijk gestopt (met toestemming van de gebruiker, alleen voor deze test — niet uit de opstartketen gehaald), `mutod.daemon` gedraaid met de echte control-loop op 25Hz. Resultaat over 600 seconden (~15.000 read-cycli): 0 gefaalde reads, 0 errors/tracebacks, exact 1 warning (de verwachte eenmalige deadman-trip vlak na start, omdat er nog geen beweging-intent was verstuurd — correct gedrag, geen bug). Na de test is `mutod` gestopt en `app_muto.py` weer herstart; de robot staat weer in exact dezelfde toestand als voor de sessie (poort weer eigendom van `app_muto.py`, PID uiteraard nieuw).

**Nog open:**
- `app_muto.py` daadwerkelijk permanent uit de opstartketen halen (`~/.config/autostart/app.desktop`) — bewust NIET gedaan deze sessie. Zolang er geen bewegingspad via `mutod` in de Nav2-launch-keten hangt, is er geen vervangende manier om de robot te besturen (bv. met een gamepad); `app_muto.py` permanent uitzetten zou nu gewoon besturing wegnemen zonder alternatief.
- `yahboom_oled.py`'s afhankelijkheid van `/tmp/battery_pct`/`/tmp/battery_volt` is een verborgen koppeling met `app_muto.py` die nergens gedocumenteerd stond — het OLED-schermpje toont straks verouderde batterijdata zodra `app_muto.py` definitief stopt, tenzij `mutod` die cache-bestanden zelf gaat bijhouden (kleine, goedkope toevoeging voor een latere fase — niet dringend).
- `muto_driver_fixed.py` (Yahboom's eigen `Hexapod`/`Movement`-IK-gait) is NIET aan `mutod` gekoppeld — zie hieronder, `phoenix_driver.py` (onze eigen gait) is in plaats daarvan gekoppeld, op uitdrukkelijk verzoek van de gebruiker.

### Fase 2b — `phoenix_driver.py` aan `mutod` gekoppeld (11 sep 2026, zelfde sessie)

Bij het uitwerken van "koppel de cmd_vel-driver aan mutod" bleek `muto_driver_fixed.py` helemaal geen STM32-firmware-gait (0x12-0x17) te gebruiken zoals de oorspronkelijke architectuurbeslissing 2 aannam, maar Yahboom's eigen host-side IK-gait (`Hexapod.move()` → `Movement`/`RealLeg`/`Servo`, dezelfde 0x40-MOTOR-writes als `mutod` al gebruikt, per servo). De gebruiker koos ervoor om in plaats daarvan **`phoenix_driver.py`** (onze eigen, al gekalibreerde tripod-gait-engine, met beter geverifieerde yaw-drift) aan `mutod` te koppelen — dat bestand bestond al, compleet met kalibratiewaarden, stop-debounce en settle-logica, maar was nog niet aan een socket of aan de Nav2-keten gehaakt.

**Aanpak: `mutod` als byte-level relay, geen IK in `mutod` zelf.** In plaats van `PhoenixGait`'s wiskunde in `mutod` te herbouwen, vervangt een dunne shim (`mutod_client.MutodSerialShim`) de `serial.Serial` die `phoenix_gait.HardwareInterface` normaal zelf opent. De shim buffert de ruwe frame-bytes die `MutoLib.Servo.motor()`/`set_exec_time()` toch al bouwen, en stuurt ze als **één** batch per gait-tick (niet 18 losse round-trips) naar `mutod`'s nieuwe `raw_frames`-op. `mutod` valideert elk frame (checksum + een korte allowlist: alleen MOTOR 0x40 en het exec-time-register 0x2C — geen vrije doorgang naar bv. RESET/TORQUE_OFF) voor het ze doorschrijft. Dit laat `phoenix_driver.py`'s gait-wiskunde, kalibratie en stop/settle-sequenties volledig ongewijzigd.

**Interferentie-mitigatie:** `phoenix_driver.py`'s eigen code-commentaar documenteerde al dat 10Hz IMU-polling op dezelfde poort merkbaar hapering gaf naast de 50Hz servo-writes. `mutod`'s eigen achtergrond-telemetrie (`read_all_angles`, normaal 25Hz) valt daarom nu automatisch terug naar 2Hz (hun eigen al-gevalideerde veilige waarde) zodra er recent bewegingscommando's binnenkomen (`gait`/`write_servo`/`write_leg`/`raw_frames`), en herstelt naar 25Hz zodra het stil is.

**Nieuwe `mutod`-ops:** `raw_frames` (batched, gevalideerde doorgeefluik) en `read_attitude` (0x60 ATTITUDE_ANGLE, roll/pitch/yaw/temp — decodering geverifieerd tegen `MutoLibCore.read_IMU()`'s formule). `phoenix_driver.py`'s eigen ad-hoc STM32-yaw-parser (`_read_stm32_yaw_deg`) is vervangen door deze `mutod`-op; er wordt niet meer los van `mutod` naar de seriële poort gelezen.

**Transport gewijzigd van Unix-socket naar TCP (127.0.0.1:8420).** De `humble_run`-container draait met `NetworkMode=host` maar heeft geen `/tmp` van de host gemount — een Unix-socket-pad zou dus onbereikbaar zijn vanuit `phoenix_driver.py`. TCP op localhost werkt in beide netwerk-namespaces identiek.

**Nieuw bestand:** `/home/pi/mutod_client.py` (én gesynchroniseerd naar `/root/mutod_client.py` in de container, zelfde handmatige sync-aanpak als `phoenix_driver.py`/`phoenix_gait.py` altijd al hadden — geen bind-mount). Bevat `MutodClient` (TCP request/response) en `MutodSerialShim`.

**`phoenix_driver.py`'s `_stop_app_muto()` is vervangen door `_require_mutod()`** — het bestand opent `/dev/myserial` nu nergens meer zelf, dus het hoeft `app_muto.py` ook niet meer zelf te killen. Het faalt nu hard met een duidelijke melding als `mutod` niet bereikbaar is op opstarttijd, i.p.v. zelf de poort te grijpen.

**Getest (zonder enige robotbeweging):**
- Volledige TCP-client/server-roundtrip live tegen de echte STM32: `read_attitude` gaf plausibele waarden (roll -1.95°, pitch 0.59°, temp 20°C).
- `raw_frames`-validatie live: een echte EXEC_TIME-registerschrijving (0x2C, interpolatietijd) ging succesvol door mutod heen naar de STM32 — dit stuurt geen enkele servo aan, puur een configuratieregister, dus geen fysieke beweging.
- Adres-allowlist en frame-checksum-afwijzing (zowel offline unit-tests als, voor de allowlist, al eerder live via de daemon-handler-tests).
- `MutodSerialShim`'s batching-logica (buffer → 1 socket-call per `flush_batch()`, foutafhandeling) offline getest met een gemockte client.

**Live bewegingstest: GESLAAGD (11 sep 2026, ~20:19).** `HardwareInterface(exec_time_ms=18, ser=shim)` opgestart binnen de `humble_run`-container (los van de volledige ROS2/rclpy-keten — alleen `MutoLib`/`phoenix_gait`/`mutod_client`, geen `rclpy` nodig voor deze test) tegen een live `mutod`. Dit stuurde alle 6 poten daadwerkelijk naar `NEUTRAL_POS` via het volledige nieuwe pad (shim → batched `raw_frames` → mutod → STM32). Gebruiker bevestigde fysiek: "poten staan netjes neutraal". IMU-attitude voor/na: roll -1.95°→-2.22°, pitch 0.56°→0.39°, yaw onveranderd (-97.45°, geen rotatie, zoals verwacht) — consistent met poten die naar een nette neutrale stand zakken.

**Onderweg naar dit resultaat: een echte bug gevangen door de raw_frames-allowlist, geen enkele beweging fout gegaan.** `phoenix_gait.py`'s `Leg.move_tip()` (via de container-eigen `MutoLib` 1.2.2-egg) bleek ALTIJD al 0x41 (LEG, gecombineerde 3-servo-frame, `Servo.set_angle_leg()`) te gebruiken — niet 3x losse 0x40 zoals aangenomen op basis van de host-egg/`MutoLibCore`. mutod's allowlist blokkeerde dit correct bij de eerste poging (hele batch geweigerd, nul bytes naar de STM32, geen beweging). Na overleg met de gebruiker is 0x41 toegevoegd aan `RAW_FRAME_ALLOWED_ADDRESSES`, specifiek gemotiveerd: dit is een ANDER, al langer gevalideerd werkend pad dan de `MutoLibCore`/`muto_hexapod_lib`-route die ooit de gedocumenteerde `OSError: No such device` veroorzaakte (Fase 1 §1.2) — geen nieuw risico geïntroduceerd, alleen een al bestaand patroon nu via mutod gerelayed. Zie `mutod/protocol.py` voor de volledige uitleg in commentaar.

**Tripod-wandeltest: GESLAAGD (11 sep 2026, ~20:22).** Los script (geen `rclpy`, direct `PhoenixGait`/`HardwareInterface`/`mutod_client`) liet de robot ~2 tripod-gaitcycli (3,2s) vooruit lopen, daarna dezelfde decel- (1s) en neutraal-inease-sequentie (1s) als `phoenix_driver.py._do_stable_stop()` gebruikt. `mutod`'s log toonde geen enkele read-fout of exception tijdens de wandeling zelf. Gebruiker bevestigde fysiek: "dit ging perfect". Hiermee is de volledige nieuwe keten (PhoenixGait-wiskunde → shim-batching → mutod raw_frames → STM32, inclusief het nieuw toegestane 0x41-pad) voor het eerst end-to-end gevalideerd met daadwerkelijke voortbeweging, niet alleen een statische pose.

**Deadman-observatie: onderzocht en gefixed (11 sep 2026, later dezelfde sessie).** Na afloop van de wandeltest vuurde de deadman's `stay_put()` twee keer meer af dan verwacht. Kon dit NIET reproduceren als een logicafout in `Deadman` zelf — geïsoleerde tests (één touch(), 25s lang geen verdere touches; ook een gelijktijdige touch()-vs-trip_once()-stresstest) toonden aan dat de originele, ongelockte versie al precies één keer afvuurde. Wat wel echt was: `Deadman`'s state werd zonder lock gelezen/geschreven vanuit meerdere threads (control-loop-thread vs. per-verbinding socket-handler-threads) — een reëel race-gat, ook al is niet met zekerheid vastgesteld dat dit specifiek de geobserveerde loganomalie veroorzaakte. Een `threading.Lock` rond `touch()`/`expired()`/`trip_once()` toegevoegd in `mutod/safety.py`, geverifieerd met een stresstest (2s lang continu gelijktijdig touchen+trippen geeft nul valse trips).

**`muto_fase1_start.sh` kent mutod nu wel (11 sep 2026, later toegevoegd).** Nieuwe STAP 0b bepaalt `$DRIVER` meteen na het killen van `app_muto.py` (i.p.v. pas bij STAP 8) en beheert mutod daarnaar: bij `DRIVER=phoenix_driver` wordt mutod gestart (of, als die al draait, met rust gelaten) en gecontroleerd op bereikbaarheid (127.0.0.1:8420) voor het script doorgaat; bij de default (`muto_driver_fixed`, opent de poort nog zelf) wordt een eventueel draaiende mutod juist gestopt om een portconflict te voorkomen. Alle vier paden (ongeldige `$DRIVER`, mutod starten, idempotent níet opnieuw starten, mutod stoppen) los getest buiten de volledige LiDAR/EKF/Nav2-keten om — geen van deze tests raakte de fysieke servo's, alleen procesbeheer. `app_muto.py` bleek er overigens vóór deze testronde al niet meer te draaien (niet door deze sessie gestopt, waarschijnlijk eerder gecrasht of handmatig afgesloten) — weer opgestart na de tests, terug in de normale toestand.

**Zijdelingse observatie, geen actie ondernomen:** `switch_to_yahboom.sh` (een apart, ouder Stack-A/Stack-B-wisselscript, niets met Fase 1/mutod te maken) controleert bij het overschakelen of `/dev/myserial` vrij is via `lsof ... | grep python`. Als mutod nog draait vanuit een eerdere `muto_fase1_start.sh DRIVER=phoenix_driver`-sessie, zou die check nu terecht mutod's eigen python-proces zien en de wissel blokkeren totdat mutod handmatig gestopt is. Dit is verwacht/correct gedrag (mutod houdt de poort inderdaad exclusief bezet), maar geen automatische afhandeling — `switch_to_yahboom.sh` zelf is niet aangepast, dat was niet gevraagd.

**Nog open na deze koppeling:**
- `phoenix_driver.py` is nog niet getest binnen de volledige ROS2/rclpy-omgeving (met `cmd_vel`-subscriptie, timers, etc.) — beide geslaagde bewegingstests eerder gebruikten alleen `HardwareInterface`/`mutod_client` rechtstreeks, zonder `rclpy`. Nu `muto_fase1_start.sh DRIVER=phoenix_driver` mutod automatisch opstart, is een volledige live test van de Nav2-keten de logische volgende stap.
- Microduck/Pollen Robotics-ecosysteem: eerste check gedaan (zie §4 Watchlist), geen directe implementatie-actie vereist nu.
- `muto_fase1_start.sh` kent `mutod` nog niet: STAP 0 killt nog steeds onvoorwaardelijk `app_muto.py`, maar start `mutod` zelf niet. Bij `DRIVER=phoenix_driver` zou `phoenix_driver.py`'s nieuwe `_require_mutod()`-check nu gewoon (correct) falen totdat `mutod` handmatig gestart is vóór dit script draait. Het script zelf is nog niet aangepast om `mutod` te starten — bewust niet gedaan, dat is een wijziging aan een bestaand, door de gebruiker onderhouden opstartscript.
- `mutod`'s TCP-poort (127.0.0.1:8420) heeft geen authenticatie — geen probleem zolang het binnen de host-network-namespace blijft (niet extern bereikbaar), maar niet vergeten als dat ooit verandert.
- Na elke test is `app_muto.py` weer teruggezet naar de normale draaiende toestand (net als bij de Fase 2-stabiliteitstest) — de robot staat nu weer in de uitgangssituatie.

**Acceptatiecriteria (herhaald ter referentie):**
- `mutod.py` start, opent `/dev/myserial` exclusief (fail loud als de poort al bezet is) — **gedaan en bevestigd, inclusief een tweede, verborgen port-gebruiker die tijdens het testen aan het licht kwam.**
- Draait een control-loop die continu `read_all_angles()` aanroept zonder te crashen (langdurige stabiliteitstest, minstens 10 minuten) — **gedaan: 10 minuten, 0 fouten.**
- `app_muto.py` staat uit (verwijderd/uitgeschakeld in de opstartketen) — **bewust nog niet gedaan, zie "nog open" hierboven; vereist eerst een vervangend besturingspad via `mutod`.**

### Fase 3 — IPC-laag (JSON-RPC, `duck-ipc-proto`-compatibel schema)

**Deliverables:**
- `mutod/ipc_types.py` — Python-dataclasses/pydantic-modellen voor `robot.move` (`vx,vy,vyaw`), `robot.state`, `robot.health`, `robot.do` (skills), `robot.stop`, `robot.subscribe`.
- `robot.move` vertaalt `(vx,vy,vyaw)` naar gait-commando's (0x12-0x17) — niet naar losse joint-writes, dat blijft voor Laag B (RL) later.
- Deadman-patroon: intent verloopt na ~600ms zonder nieuw bericht (patroon uit `meckie-duck-gateway`, zie eerdere sessie). **Basisversie hiervan bestaat al in `mutod/safety.py` (Fase 2) — Fase 3 breidt uit naar het volledige IPC-schema.**
- Testtegenpartij: `joeynyc/microduck-mcp`'s `mock`-transport — valideer dat `mutod`'s berichten door een onafhankelijke, actueel-gehouden referentie-implementatie worden herkend.

**Acceptatiecriteria:**
- Twee lokale processen (mock-microduck-client ↔ `mutod`) wisselen `robot.move`/`robot.state`/`robot.health` succesvol uit.
- Deadman-timeout aantoonbaar: stop met berichten sturen → robot stopt vanzelf.

**Status 12 sep 2026: gebouwd, offline en live getest, inclusief een echte bewegingstest.** `mutod/ipc_types.py` (nieuw) + uitbreiding van `daemon.py`'s TCP-handler: herkent nu ook echte JSON-RPC 2.0-berichten (naast, niet i.p.v., Fase 2's kortere `{"op": ...}`-vorm die `phoenix_driver.py`/`mutod_client.py` blijft gebruiken). Methode-namen/vorm rechtstreeks opgezocht in `joeynyc/microduck-mcp`'s `src/transport/protocol.ts` (de geplande testtegenpartij) i.p.v. aangenomen.

Geïmplementeerd: `robot.health`, `robot.state`, `robot.move`, `robot.stop`, `robot.subscribe` (achtergrond-pushthread per verbinding). `robot.do` bestaat als plumbing maar wijst elke skill af (`INVALID_PARAMS`) tot Fase 6 `0x3E`/ACTION daadwerkelijk valideert — bewust geen ongeteste hardware-aanroepen via een achterdeur.

**Getest:**
- Offline (`mutod_ipc_selftest.py`, fake HAL, geen hardware): 20/20 checks — methode-dispatch, vx/vy/vyaw→richting, deadband, validatie, Fase 2-kanaal ongewijzigd.
- Live tegen de echte draaiende daemon (`mutod_ipc_live_check.py`): `robot.health`/`robot.state`/`robot.subscribe`/`robot.stop` — hierbij een echte race condition gevonden en gefixed (de eerste `robot.state`-notificatie kon de `robot.subscribe`-bevestiging op de socket inhalen doordat de pushthread al binnen `_handle_rpc` startte; nu start `Handler.handle()` de thread pas na het schrijven van de bevestiging).
- **Live bewegingstest: GESLAAGD (12 sep 2026).** Eén `robot.move {vx:0.021}`-bericht, verstuurd door de gebruiker zelf in een terminal (de auto-mode-classifier blokkeerde dit net als bij de Fase 1-launch eerder). Resultaat: `/odom_fused` Δx=+0,024m/Δy=-0,003m, oriëntatie vrijwel ongewijzigd, precies één deadman-trip in `mutod`'s log (stopte vanzelf na ~0,6s, geen fouten). Gebruiker bevestigde fysiek: "klein stapje naar voren, richting klopt" — dit is de EERSTE keer dat de STM32-firmware-gait (0x12-0x17) via `mutod` daadwerkelijk beweging veroorzaakt heeft; FORWARD/vx-tekenconventie is nu empirisch bevestigd voor dit pad.

**SHIFT/TURN apart geverifieerd, zelfde sessie (12 sep 2026):** twee losse `robot.move`-tests (alleen `vy`, dan alleen `vyaw`, elk eenmalig en deadman-begrensd). Resultaat: `vy>0` → robot stapt naar zijn EIGEN links (bevestigd door de gebruiker, expliciet nagevraagd of dit robot-eigen of waarnemer-links was), `vyaw>0` → robot draait naar zijn EIGEN links — beide exact zoals `move_to_gait()`'s bestaande aannames al veronderstelden, dus GEEN codewijziging nodig. Alle vier bewegingsrichtingen (FORWARD/BACKWARD/SHIFT_LEFT/SHIFT_RIGHT/TURN_LEFT/TURN_RIGHT) zijn nu empirisch bevestigd voor dit specifieke pad. De step-naar-snelheid-schaling (`MOVE_REFERENCE_MAX_MPS`/`_RADPS`) blijft wel bewust NIET snelheidsgekalibreerd, puur functionele plumbing.

### Fase 4 — Discovery & coördinatie (UDP-beacon)

**Deliverables:**
- `mutod/discovery.py` — UDP-broadcast beacon (robot-ID, capabilities, ip:poort, tijdstempel), interval ~1,5s.
- Settle-window (1,5s) en stale-timeout (3s), deterministisch leiderschap (laagste ID), naar analogie van chorale — eigen payload, niet hun muziek-specifieke een.
- **Opt-in default-uit**: een Muto reageert pas op beacons van andere robots als dat expliciet is ingeschakeld.
- Testbaar met twee lokale processen op verschillende UDP-poorten (geen tweede robot nodig).

**Status 12 sep 2026: gebouwd en getest, geen hardware/beweging bij betrokken.** `mutod/discovery.py` (nieuw): `PeerRegistry` (first_seen/last_seen per robot_id, thread-safe, `time.monotonic()`-gebaseerd), `elect_leader()` (laagste ID wint, self altijd meegeteld), `DiscoveryService` (UDP-broadcast op poort 8421, apart van mutod's TCP-kanaal 8420). Volledig **opt-in**: er wordt geen socket geopend en geen enkel pakket verstuurd/ontvangen totdat `start()` expliciet aangeroepen wordt (geen aparte "alleen luisteren"-stand — zolang er niets is om mee te coördineren voegt dat alleen LAN-ruis toe).

**"Chorale" zelf niet teruggevonden:** een publieke referentie-implementatie met die naam leverde bij zoeken (12 sep) alleen muziek/Bach-chorale-datasets op, geen swarm-coordinatieproject — vermoedelijk een niet-publieke of anders geheten bron uit een eerdere sessie. De architectuurbeslissing zelf specificeerde de parameters al concreet genoeg (settle 1,5s, stale 3s, laagste-ID-leiderschap) om zonder die bron te bouwen. Als de originele bron ooit teruggevonden wordt: dit bestand er alsnog tegen afzetten.

**Getest:**
- Offline (`mutod_discovery_selftest.py`, synthetische tijdstempels, geen sockets): settle-window, stale-timeout, prune(), deterministisch leiderschap (verschillende peer-combinaties), first_seen blijft ongewijzigd bij herhaalde updates — 13/13 checks.
- Live (`mutod_discovery_live_check.py`): **twee echte `DiscoveryService`-instanties, elk op zijn eigen echte UDP-socket** (via `SO_REUSEPORT` op dezelfde broadcast-poort, lokaal getest zonder tweede robot) — vinden elkaar daadwerkelijk, respecteren het settle-window, kiezen onafhankelijk dezelfde leider (laagste ID), en A laat B correct los na diens stop + stale-timeout, waarna A weer zichzelf als leider ziet. 12/12 checks.

**Nog niet gedaan (bewust, buiten scope van Fase 4's eigen deliverables):** geen koppeling met `mutod/daemon.py` of de IPC-laag (bv. peer/leider-info via `robot.health`) — dat is een natuurlijk aanknopingspunt voor Fase 5, niet expliciet gevraagd voor Fase 4 zelf.

### Fase 5 — Gedragslaag (state machine)

**Deliverables:**
- `mutod/behavior.py` — centrale state machine met vier toestanden: Rust, Rondlopen, Interactie, Laag batterij. Eén beslisser die batterijniveau/tijd-sinds-interactie/aanwezigheid van andere robots als *input* neemt, niet losse if-else-ketens.
- **Rondlopen: novelty-grid-wandelroutine** — visit-count-raster (cel ~20cm) op basis van odometrie, richtingskeuze op LiDAR-vrije-ruimte × onbezocht-score. Geen Nav2-afhankelijkheid nodig.
- `robot.state`'s `mode`-veld rapporteert de huidige toestand naar buiten (koppelt aan fase 3's IPC).

**Status 12 sep 2026: gebouwd en getest — GEEN autonome beweging uitgevoerd of ingeschakeld.** `mutod/behavior.py` (nieuw): `BehaviorStateMachine` (vier toestanden, prioriteit batterij > interactie > andere-robots-aanwezig > idle-timeout-naar-Rondlopen, met hysterese rond de batterij-drempel zodat het niet flippert), `NoveltyGrid` (visit-count per ~20cm-cel) + `choose_heading()` (kiest de kandidaat-richting met de beste `min(clearance,2.0) × onbezocht-score`, retourneert `None` als niets voldoende vrij is — nooit een default-richting forceren bij twijfel).

**Bewuste scope-grens:** dit bestand levert alleen de BESLISLOGICA + de `mode`-rapportage via `robot.state` (`daemon.py` ticked de state machine nu bij elke `robot.state`-opvraag, met batterij uit dezelfde cache als `robot.health` en `peers_present` hardcoded op `False` tot Fase 4's `DiscoveryService` bewust gekoppeld wordt). Er wordt vanuit `behavior.py`/de daemon GEEN enkele `hal.gait()`-aanroep gedaan — het daadwerkelijk laten rondlopen in WANDER-modus (onbeheerde, voortdurende, zelf-gekozen beweging) is bewust een aparte, nog niet aangevraagde/aangekondigde stap, veel groter van impact dan een los `robot.move`-commando.

**Getest, geen hardware/beweging bij betrokken:**
- Offline (`mutod_behavior_selftest.py`, injecteerbare fake klok): 22/22 checks — alle state-overgangen (incl. batterij-hysterese en de peers-present-onderdrukking van Rondlopen), `NoveltyGrid`-scores, `choose_heading()`'s selectie- en "niets vrij"-gedrag.
- Live: `robot.state` geeft nu daadwerkelijk `mode` terug over de echte socket (bevestigd: `"rest"` direct na een herstart van `mutod`), zonder dat dit enige beweging veroorzaakte.

### Fase 6 — Extra skills (goedkope wins, test eerst vóór bouwen)

- **Voorpoot-zwaai**: test eerst 0x3E (ACTION, performance-groepen) op "greet"/"wave" — mogelijk al klaar zonder nieuwe code. Zo niet: nieuwe skill, gated op stilstand (twist_magnitude==0 + stand-houding), analoog aan Microducks eigen prioriteitsketen.
- **Zijwaartse gait**: test eerst of `phoenix_gait.py`'s wereldframe-translatie (na de mount-hoek-fix) al generaliseert naar zijwaartse richting — waarschijnlijk grotendeels gratis. Terugval: ingebouwde `SHIFT_LEFT`/`SHIFT_RIGHT` (0x14/0x15).
- **Contact-detectie herbouwen** op 0x50 (bulk-read, servo-positiefout) i.p.v. het foutieve 0x60-gebruik in de huidige `foot_contact.py`.
- **Camera-gerichtheid tijdens interactie** — bestaande Yahboom face/color-detection, geen nieuwe CV-stack nodig.

**Status 12 sep 2026:**

1. **Contact-detectie: KLAAR, live getest.** [foot_contact.py](../../../../foot_contact.py) volledig herbouwd — leest nu via `mutod`'s TCP-kanaal (`{"op":"get_state"}`, dezelfde 0x50-bulk-read als de control-loop) i.p.v. een eigen `serial.Serial()` op 0x60 (die nu sowieso EBUSY zou geven, mutod is exclusief poorteigenaar). Stuurt zelf geen enkel bewegingscommando meer. Live gecheckt: geeft zinvolle tibia-hoeken terug.

2. **Voorpoot-zwaai (ACTION/0x3E): plumbing klaar + fysiek getest.** Complete parameter-mapping gevonden in `MutoLibCore.py`'s `action()`-docstring (ground truth, niet afgeleid): 0=reset, 1=stretch, **2=greet/hello**, 3=fear/retreat, 4=warmup, 5=spin-in-place, **6=wave/"no"**, 7=curl-up, 8=stride-forward. Toegevoegd: `mutod/hal.py`'s `action()` + `daemon.py`'s `"action"`-op + `protocol.py`'s `ACTION_NAMES`. **Live getest (action_id=2, "greet"):** liep zonder fouten, precies één verwachte deadman-trip. Gebruiker bevestigde: "poten kort bewogen, verder niets gebeurd. poot staat nog wel omhoog" — de routine laat dus een been omhoog staan, `stay_put()` corrigeert dat niet vanzelf. Robot bleef stabiel. Opgelost met een losse `reset_posture`-aanroep erna (bevestigd: alle poten weer omlaag). **Belangrijk voor de latere skill-wrapper:** een `robot.do`-skill die ACTION gebruikt moet zelf een `reset_posture()` na afloop inbouwen, niet aannemen dat de routine zelf netjes naar neutraal terugkeert. `action_id=6` ("wave/no") is nog niet uitgeprobeerd.

3. **Zijwaartse gait: plumbing klaar + fysiek getest, ÉÉN keer bijgesteld na de test.** Bevestigd in de code: `PhoenixGait.foot_targets()` had al een eigen `travel_z`-as (in `_foot_delta()`) die nooit vanuit `phoenix_driver.py` werd aangestuurd (`cmd_vel.linear.y` werd genegeerd, altijd `travel_z=0.0`). Eerste testpoging via een directe `ros2 topic pub` naar `/cmd_vel` **kwam nergens aan** (`phoenix_driver` registreerde geen enkele state-wijziging) — vermoedelijke oorzaak: de test miste `CYCLONEDDS_URI=file:///root/cyclone_dds.xml`, dus zat mogelijk in een ander DDS-domein dan de node zelf; geen robotbeweging opgetreden. Tweede poging via een los, direct script (`phoenix_sideways_test.py`, zelfde bewezen patroon als `phoenix_yaw_drift_test.py` maar via `MutodClient`/`MutodSerialShim` i.p.v. een eigen seriële poort, en met `phoenix_driver.py` tijdelijk gestopt om overlap te voorkomen): 1 voorzichtige gaitcyclus op 60% snelheid, **werkte meteen**. Gebruiker bevestigde: `travel_z=+1` → robot-EIGEN RECHTS. Dat is het omgekeerde van de standaard ROS-conventie (`linear.y>0`=links) — omdat nog niets van de nieuwe koppeling afhing, is het teken in `phoenix_driver.py`'s `cb()` omgedraaid (`travel_y = clamp(-msg.linear.y / ..., ...)`), zodat `cmd_vel.linear.y` voortaan wél de standaard conventie volgt. Bijgewerkt, gesynchroniseerd naar de container en `phoenix_driver` herstart.

4. **Camera-gerichtheid: perceptie-deel KLAAR en live bevestigd; besturing (echt draaien) nog niet gebouwd.**

   **Grote vooruitgang t.o.v. de eerdere inschatting:** de camera bleek een Orbbec Astra (Astra Pro-variant: aparte Sonix-RGB-sensor + Orbbec-dieptesensor, twee losse USB-devices) met een AL EERDER GEBOUWDE `ros2_astra_camera`-driver op het host-filesystem (`/home/pi/yahboomcar_ros2_ws/software/library_ws_humble/install/astra_camera`, build van 4 dec 2025 — bevestigt weer eens [[muto_check_local_before_external]]: lokaal checken voor je iets als "moet nog gebouwd worden" bestempelt). Deze driver was echter nog nooit gestart. `astra_pro.launch.xml`'s default `uvc_product_id` (`0x0501`) matchte niet het echte toestel (`2bc5:050f` volgens `lsusb`) — met `uvc_product_id:=0x050f` opende de kleurenstroom meteen succesvol. `/camera/color/image_raw` (rgb8, 640x480, ~13,5Hz) bevestigd live, een frame opgeslagen en visueel gecontroleerd: een echte, scherpe foto.

   **[face_tracker.py](../../../../face_tracker.py) (nieuw):** een ROS2-node die MediaPipe-gezichtsdetectie (Yahboom's bestaande `FaceDetector`-aanpak, hier hergebruikt tegen de echte cameratopic i.p.v. hun eigen `cv.VideoCapture(0)`-demo) draait op `/camera/color/image_raw`, en de horizontale richting (`bearing_deg`, berekend uit de echte camera-intrinsics `fx`, HFOV~60,7°) publiceert op `/face_bearing` als JSON. **Bewust alleen waarneming — deze node stuurt zelf geen enkel bewegingscommando.**

   **Teken-conventie live bevestigd (12 sep 2026):** gebruiker recht voor de camera → bearing ~0°. Gebruiker naar zijn/haar eigen rechts gestapt, tegenover de camera aankijkend → bearing -15,5° (negatief). Aangezien de camera niet spiegelt, betekent dat: **negatieve bearing = persoon staat robot-eigen LINKS**, positieve bearing = robot-eigen RECHTS. Consistent met de al bevestigde `vyaw>0`=robot-links-conventie (Fase 3): een toekomstige stuurlus zou `gewenste_vyaw = -k × bearing_deg` kunnen gebruiken.

   **Stuurlus GEBOUWD EN LIVE BEVESTIGD (12 sep 2026, zelfde sessie, vervolgstap):** [face_follow_controller.py](../../../../face_follow_controller.py) — sluit de lus tussen `/face_bearing` en een echt `robot.move`-draaicommando (via mutod's Fase 3 IPC, dezelfde firmware-gait-TURN-weg als de eerder bevestigde TURN-test). Veiligheidsontwerp: `--dry-run`-default (pas `--live` stuurt echt), dead-band (8°), rate-limit (1 correctie/~0,6s, niet op elke cameraframe), geklemde correctiesnelheid (max ±0,10, zelfde ordegrootte als de bevestigde TURN-test), en een 15s-watchdog die zichzelf stopt bij te lang onafgebroken corrigeren zonder centrering. Stuurt altijd een expliciete `robot.stop()` bij verlies van het gezicht, centrering, of de watchdog.

   Eerst dry-run getest (gebruiker links vanuit de robot → correct berekend `vyaw=+0,096`=TURN_LEFT, stopte netjes bij het (kort) wegvallen van het gezicht — een reële beperking hierbij: gezichtsdetectie flikkert snel aan/uit bij een zijwaartse/schuine hoek, geen bug maar een MediaPipe-frontale-detectie-limiet). Daarna **live getest, expliciet aangekondigd en bevestigd door de gebruiker**: robot draaide daadwerkelijk en correct naar de gebruiker toe bij meerdere zijwaartse posities, stopte netjes bij centrering/verlies van het gezicht. Gebruiker: "draaide echt netjes naar me toe, richting klopt". Na de test bewust weer gestopt (niet onbeheerd laten doorlopen).

   Hiermee is Fase 6 punt 4 volledig afgerond (perceptie + besturing, beide live bevestigd) — als eerste closed-loop, camera-reactieve gedraging in dit project.

**Vervolg 12 sep 2026, na architectuurbeslissing 7 (SLAM/AMCL geparkeerd):** Nav2/AMCL-stack expliciet gestopt (niet langer standaard nodig). [depth_obstacle.py](../../../../depth_obstacle.py) (nieuw) geeft Fase 5's novelty-grid-wandelroutine een tweede obstakelsensor naast LiDAR: zet een ruwe Astra-dieptebeeld om naar dezelfde `{richting_graden: afstand_m}`-vorm die `choose_heading()` van LiDAR verwacht, plus `merge_lidar_and_depth_clearance()` (minimum waar beide overlappen, LiDAR blijft leidend buiten de camera's smallere gezichtsveld). 10/10 offline checks (synthetische depth-frames) + live bevestigd tegen de echte camera (`{-15°: 1.79m, 0°: 1.26m, 15°: 1.74m}`, realistische waarden).

**Wander-executor gebouwd, live getest, ÉÉN ECHT VEILIGHEIDSINCIDENT (12 sep 2026).** [wander_executor.py](../../../../wander_executor.py) (nieuw) combineert `lidar_obstacle.py` (nieuw, LiDAR-clearance-per-richting) + `depth_obstacle.py` + `choose_heading()`/`NoveltyGrid` tot echte, doorlopende `robot.move`-aansturing. Eerste bug (besluiteloos heen-en-weer draaien i.p.v. commit'en aan een richting) gevonden en gefixed via een "vastgehouden doel"-mechanisme, dry-run + live bevestigd werkend (robot liep daadwerkelijk en correct vooruit).

**Vervolgens een echt veiligheidsincident:** tijdens die geslaagde voorwaartse loop liep de robot ~50s door richting een kast recht vooruit zonder te stoppen — de gebruiker moest hem fysiek verplaatsen. Root cause via een live `tf2_echo`-check (niet gegokt): de front-veiligheidscheck gebruikte een `+180`-correctie geleend van `lidar_overlay.py`'s `STATIC_YAW`, maar die geldt voor de frame `laser_scan_fix` — `/scan_fixed`'s werkelijke frame is `laser`, een ANDER frame, waarvoor `base_link->laser` in werkelijkheid 0° rotatie is (bevestigd live). Zelfs zonder die foute correctie klopte de hoek van de dichtstbijzijnde meting nog niet overtuigend met "recht vooruit" — de echte hoek-relatie tussen rauwe scan-hoek en de werkelijke looprichting is dus **nog niet betrouwbaar vastgesteld**, zie [[muto_wander_frontcheck_bug_2026-09-12]] voor het volledige verhaal.

**Tijdelijke fix (defense-in-depth, geen echte kalibratie):** elke voorwaartse stap (en een top-level check vóór elke actie) wordt nu gegate op het **globale minimum over alle gedetecteerde richtingen samen**, niet een vermoede "voorkant"-richting-bucket — conservatiever (kan ook stoppen voor iets opzij), maar veilig ongeacht of de richting-toewijzing zelf klopt. Live herbevestigd met de robot nog naast de kast: de nieuwe gate vuurde meteen correct af (min=0,22m).

**Openstaand:** een echte hoek-kalibratie tussen rauwe LiDAR-scan-hoek en de werkelijke looprichting is nog niet gedaan — de exploratie-richtingkeuze (welke kant op draaien) kan hierdoor nog steeds verkeerd gericht zijn, al is dat nu geen botsingsrisico meer (alleen inefficiëntie).

### Fase 7 — Laag B: RL / joint-level control (grotere, latere stap)

**Voorwaarden vóór starten:** fase 2 (`mutod.py`) stabiel en getest, en een bewuste keuze dat joint-level RL-control daadwerkelijk gewenst is naast de gait-commando-aanpak van fase 2-6 (dit is een fundamenteel andere besturingsstijl, geen incrementele uitbreiding).

- Obs/actie-schema ontwerpen voor een hexapod (18 joints, dus obs/actie-dimensies anders dan Microducks 61/14) — lichaamshoogte/roll/pitch invullen op hexapod-manier.
- Muto-MJCF bouwen (CAD/meetgegevens nodig, nog niet aanwezig).
- `microduck_rl` forken, robotmodel vervangen, **`env.step`-tijd aanpassen aan Muto's ~40-50ms** (niet Microducks 20ms — zie §1.4, dit is een keiharde correctie op de standaardconfiguratie).
- Trainingsstack (mjlab, WSL2/RTX 5080) is al bevestigd werkend — geen installatieproblemen te verwachten.
- Inferentie: TensorRT overwegen i.p.v. ONNX Runtime, gezien de Jetson Orin Nano in de stack (Microduck heeft dit niet nodig, wij mogelijk wel).

### Fase 8 — Multi-robot met echte Microduck (na levering, ~december 2026)

- Bearing-naar-Microduck via camera (klein eigen detectiemodel, analoog aan `duck-detect`) als aanvulling op UDP-discovery, die geen richting geeft.
- Chorale-equivalent testen tegen de echte Microduck-hardware i.p.v. alleen mock-transports.

---

## 4. Watchlist (doorlopend, geen aparte fase)

**Status 11 sep 2026: eerste volledige check uitgevoerd (was nog nooit eerder gedaan sinds het aanleggen van deze lijst).** Het ecosysteem is groot en snel bewegend — awesome-microduck alleen al somt tientallen projecten op. Onderstaand alleen wat daadwerkelijk relevant is voor Muto's eigen roadmap; de volledige lijst staat op [joeynyc/awesome-microduck](https://github.com/joeynyc/awesome-microduck), niet hier gedupliceerd.

**Direct relevant voor Fase 7 (RL) — nu er een STEP-bestand van Muto is (zie hieronder):**
- [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) — actief (1215 commits, 2.1k sterren), mjlab (MuJoCo Warp) + PPO, 50Hz-training. Bevat een direct herbruikbare sim2real-aanpak: BAM-actuatormodel (voltage/back-EMF/Coulomb-Stribeck-wrijving) i.p.v. een naief servo-model, domain randomization op batterijspanning/commandovertraging/wrijving, en ±1° backlash-simulatie in serie met elk gewricht. Dit zijn technieken om over te nemen bij het bouwen van Muto's eigen sim2real-pijplijn, niet per se code om te forken.
- [Isaac Lab Microduck-port](https://github.com/joeynyc/awesome-microduck) (BAM-actuators + RSL-RL taken) — een tweede, onafhankelijk gevalideerd voorbeeld van hoe je een robotmodel + actuatormodel naar een trainingsframework overzet. Nuttig als tweede referentie naast mjlab, mocht mjlab op de WSL2/RTX5080-stack ooit problemen geven.
- [pollen-robotics/microduck-policies](https://hf.co/pollen-robotics/microduck-policies) (HF, bijgewerkt 2 sep 2026) — laat zien hoe een afgeronde policy eruitziet (policy.onnx + normalizer + manifest.json) — nuttig als structuurvoorbeeld voor Muto's eigen export, ook al is het beest een tweevoeter en Muto een hexapod.

**Direct relevant voor Fase 3 (IPC-testtegenpartij):**
- [joeynyc/microduck-mcp](https://github.com/joeynyc/awesome-microduck) — bevestigd nog actief, transports: mock/MuJoCo-sim/Unix-socket/SSH. Blijft de aangewezen testtegenpartij zoals gepland.
- Nieuw ontdekt: een TWEEDE, onafhankelijke MCP-server-implementatie (`aj-dev-smith/microduck-mcp`, gesimuleerde eend met camera-frames als tool-output) — niet per se nodig, maar een tweede referentie-implementatie als joeynyc's versie ooit stil valt.

**Direct relevant voor Fase 8 (bearing-naar-Microduck):**
- [pollen-robotics/microduck-duck-detector](https://hf.co/pollen-robotics/microduck-duck-detector) (HF, 10 sep 2026 — letterlijk gisteren) — object-detector die andere eenden herkent (0.80 mAP50), YOLO/ONNX/RKNN. Exact het `duck-detect`-analoge model dat Fase 8 al noemde als te bouwen — nu bestaat er dus al een direct kopieerbaar voorbeeld/architectuur (al train je op Muto's eigen uiterlijk, niet op eenden).

**Al gebruikt, ter bevestiging nog steeds correct:**
- `meckie-duck-gateway` — het deadman/intent-timeout-patroon (~600ms) dat al in `mutod/safety.py` zit, komt hiervandaan; bevestigd nog actueel.
- `quackd` — LLM-goal-planning, relevant naast de bestaande Dify-integratie, nog niet met prioriteit opgepakt.

**Kennisgenomen, geen actie nu:** `microduck-simulator` (HF Space, 502 likes, browser-MuJoCo-sim), `microduck-emotions` (HF dataset, motion+sound-combinaties — mogelijk inspiratie voor Fase 6's gebaren/skills), `microduck-console` (WebRTC remote control), en tientallen community-projecten (backflip/courier/stilts/detectors/edge-ports/etc.) opgesomd in awesome-microduck — niets hiervan is nu urgent, wel de moeite waard om te kennen als er ooit een vergelijkbare behoefte ontstaat.

- Yahboom: geen updates te verwachten (bevestigd, definitief) — niet blijven checken.

---

## 5. Wat NIET te doen

- Geen `leg_motor()`/0x41 gebruiken in nieuwe code — altijd 3× `motor()`/0x40.
- Geen 0x60 gebruiken voor servo-feedback — het is IMU.
- Geen 50Hz-controlrate aannemen zonder de 27ms-bulk-read-limiet te respecteren.
- Geen BLE-stack bouwen voor discovery — UDP volstaat voor het LAN-scenario.
- Niet wachten op Yahboom voor firmware-updates — die komen niet.

---

## 6. Update 13 sep 2026 (later in de dag)

**`app_muto.py`'s autostart is nu permanent uitgeschakeld** (sluit het "nog open"-punt uit sectie hierboven, 11 sep 2026 -- `~/.config/autostart/app.desktop` hernoemd naar `app.desktop.disabled`, herstart-bestendig geverifieerd met een echte reboot). Reden dat dit nu wel kon, in tegenstelling tot 11 sep: `mutod` is inmiddels een volledig uitgebouwd en live-geteste besturingspad (Fase 3-7), dus de oorspronkelijke blokkerende reden ("geen vervangende manier om de robot te besturen") is niet meer van toepassing.

**Bijwerking opgelost, zelfde dag:** `yahboom_oled.py`'s autostart (`~/.config/autostart/oled.desktop`) is nu ook permanent uitgeschakeld (zelfde reversibele methode: hernoemd naar `oled.desktop.disabled`), na afweging dat het laten draaien juist het risico was (terugval op een eigen `/dev/myserial`-verbinding die met `mutod` kan botsen), niet het uitzetten. Enige gevolg: het fysieke OLED-batterijschermpje blijft leeg -- geen functioneel verlies voor besturing/navigatie. `/dev/myserial` heeft nu geen enkele andere claimer meer dan `mutod`.

**Camera-diepte-interface (Orbbec Astra, `2bc5:060f`): root cause herbevestigd, geen nieuwe info.** Zelfde patroon als eerder (zie Fase 7/camera-sectie): bij twijfelachtige fysieke verbinding (hub i.p.v. directe Pi-poort) enumereert alleen de kleur-interface (`2bc5:050f`), de diepte-interface komt helemaal niet in `dmesg` voor. Bevestigd met een schone reboot (geen verandering) en pas opgelost na een echte fysieke kabel-reconnect (bevestigd via een nieuwe `dmesg`-regel voor `2bc5:060f`). Blijf dit checken als eerste stap als de camera ooit weer geen diepte geeft.

**Audio/TTS: volledig opgelost en uitgebreid.** De USB-speaker bleek nooit kapotte hardware — een udev-regel (`99-yahboom-audio.rules`) blokkeerde de ALSA-driver, verwijderd en bevestigd na reboot. TTS vervolgens toegevoegd aan `yolo_snapshot_sender.py` (`--speak`-flag): eerst espeak-ng geprobeerd (Nederlands en Engels, ook met lagere spreeksnelheid) — user-oordeel steeds "slecht"/"matig" verstaanbaar. Vervangen door **Piper** (lokale neurale TTS, `en_US-amy-medium`-stem, via een pip-venv omdat dit OS `pip install` systeembreed blokkeert) — user-bevestigd "veel beter!!". espeak-ng volledig verwijderd. Geinspireerd door onderzoek naar Reachy's (Pollen Robotics) eigen TTS-aanpak (cloud-gebaseerd, Deepgram/Grok Voice) — Piper is het lokale/offline equivalent zonder cloud-afhankelijkheid.

**Mijlpaal, zelfde dag:** eerste succesvolle autonome deur-doorgang met `wander_executor.py` (corridor-veiligheid, watchdog-fix, doel-volg-prioriteit, corridor-centreren, kleurgebaseerde detectie i.p.v. helderheid-gebaseerd — zie de code-comments in `wander_executor.py`/`behavior.py` voor details per fix).
