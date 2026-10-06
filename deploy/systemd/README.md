# systemd-eenheden voor Muto

| Eenheid | Wat |
|---|---|
| `mutod.service` | `mutod`, enige eigenaar van `/dev/myserial`. Herstart na 5 s bij een fout en blijft het proberen als de USB-seriele poort bij het booten nog ontbreekt. |
| `muto-voice-listener.service` | `skills/voice_stop_listener.py` ("muto" + "stop" -> `robot.stop`). Start na mutod, draait met de groep `audio`. |

Installeren (eenmalig, op de Pi):

```bash
sudo cp deploy/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mutod.service muto-voice-listener.service
```

Let op:
- Start mutod of de listener daarna NIET ook met de hand: de seriele poort en de microfoon zijn exclusief (een tweede instantie stopt direct).
- De paden gaan uit van gebruiker `pi`, `/home/pi` en de venv `/home/pi/whisper_venv`; pas ze aan als je een andere indeling hebt.
- Logs: `sudo journalctl -u mutod -f` en `sudo journalctl -u muto-voice-listener -f`.
- Een harde `kill -9` van elk van beide is getest (6 okt 2026): ze komen binnen ~6 s terug. Een echte herstart van het Pi is ook getest (6 okt 2026): beide eenheden kwamen vanzelf op.
- De listener logt zonder `--debug` alleen gebeurtenissen en niet de herkende tekst (journald is permanent; spraak uit de kamer hoort daar niet in).
