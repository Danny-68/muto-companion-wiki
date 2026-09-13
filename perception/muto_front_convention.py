#!/usr/bin/env python3
"""Enige, centrale bron van waarheid voor de relatie tussen base_link's
URDF-conventie (+x, gebruikt door AMCL/Nav2/EKF/TF) en de fysieke voorkant
van de robot.

Waarom dit bestand bestaat (21 aug 2026, avond): eerder stond deze correctie
los, hardcoded en ongedocumenteerd in slechts één script (lidar_overlay.py's
FRONT_DISPLAY_OFFSET_DEG). Daardoor kon "kijkrichting" en "wat de LiDAR/URDF
als voorwaarts beschouwt" in de praktijk twee losse, uit elkaar lopende
variabelen worden -- precies de onbetrouwbaarheid die dit bestand voorkomt.
Elk script dat de fysieke voorkant moet tonen/gebruiken importeert dit
bestand, in plaats van een eigen kopie van de constante te hardcoden.

Vastgesteld via front_scan_check.py (21 aug 2026): een kaart/AMCL-vrije test
die puur de live LiDAR-scan in base_link-frame tekent (met alleen de vaste,
fysiek geverifieerde laser->base_link montage-transform, zie Muto.urdf's
LD_Joint + laser_scan_fix_joint) -- de gebruiker bevestigde direct dat
base_link's eigen +x-as precies 180 graden omgekeerd is t.o.v. de fysieke
voorkant. Dit is dus een vaste, structurele chassis/URDF-modelleerkeuze,
GEEN AMCL- of omgevingsafhankelijk effect (die test gebruikte geen kaart en
geen AMCL-pose).

Wat dit WEL en NIET aanraakt:
- WEL: elke tekening/weergave die "dit is de fysieke voorkant" claimt.
- NIET: de scanpunten zelf, de yaw die naar AMCL/Nav2/EKF gaat, of enige
  rijrichting-berekening -- die blijven allemaal in base_link's eigen,
  interne conventie, want die functioneren al onderling consistent (de
  eerdere geslaagde NavigateToPose bewijst dat rijrichting en URDF-conventie
  elkaar al correct vinden).

Als de URDF ooit zelf gecorrigeerd wordt (base_link's +x fysiek omgedraaid,
met alle bijbehorende kind-joints herijkt), moet FRONT_YAW_OFFSET_DEG hier
naar 0.0 -- en dan werkt elk script dat dit bestand importeert vanzelf mee,
zonder los overal te hoeven zoeken.
"""
import math

FRONT_YAW_OFFSET_DEG = 180.0
FRONT_YAW_OFFSET_RAD = math.radians(FRONT_YAW_OFFSET_DEG)


def base_link_yaw_to_physical_front(base_link_yaw_rad):
    """Rekent een yaw uitgedrukt in base_link's eigen (URDF-)conventie om
    naar de richting waar de robot fysiek daadwerkelijk naar toe kijkt."""
    return base_link_yaw_rad + FRONT_YAW_OFFSET_RAD
