# ARC-Core

ARC-Core ist die gemeinsame Firmware-Bibliothek (PlatformIO, Arduino, ESP32-S3) von
[ARC-Play](https://github.com/probstj03/ARC-Play) und [ARC-Slate](https://github.com/probstj03/ARC-Slate).
Sie liefert eine RTC-gestützte Zeitbasis, Timecode und Synchronisation. Ziel ist, dass alle ARC-Geräte
nach einem Jam Sync **subframe-genau** synchron laufen. Gefordert ist eine Abweichung von höchstens
1/10 Frame, angestrebt sind unter 0,2 ms.

## Module

| Datei | Inhalt |
|---|---|
| `arc_timecode` | Bildraten 24/25/29.97 DF/30, Drop-Frame-Umrechnung, Frame ↔ Echtzeit (exakte Ganzzahl-Rechnung) |
| `arc_timeline` | TC als Funktion der Wanduhrzeit: Free Run, verankert (Record Run / Playback, mit Geschwindigkeit), gestoppt |
| `arc_userbits` | Shot-Userbits (Kamera/Roll/Szene/Shot/Take, passend zu ARC-Server), Datum + Zeitzone, EU-Sommerzeit |
| `arc_clock` | RTC-gestützte Wanduhr (Details siehe unten) |
| `arc_ltc_gen` | phasengeregelter LTC-Generator + `OutputClock` (Details siehe unten) |
| `arc_ltc_in` | LTC-Decoder mit µs-Zeitstempel je Frame-Grenze |
| `arc_engine` | Modi, Transport mit geplanter Ausführung (`at`), Jam Sync, Latenz-Kalibrierung (Details siehe unten) |
| `arc_net` | bekannte WLANs (NVS) → ARC-Net → eigenes ARC-Net, mDNS, WebSocket-Protokoll v2 |
| `arc_events` | Ereignis-Log in LittleFS (`/events.jsonl`) |

**`arc_clock`:**
- Die Uhr arbeitet mit der Sekundenflanke des DS3231 (SQW) und interpoliert dazwischen mit dem ESP-Timer, korrigiert um dessen Gangabweichung.
- Welche SQW-Flanke den Sekundenwechsel markiert, erkennt sie automatisch.
- Nach dem Stellen gleicht ein Software-Trim die Übernahmeungenauigkeit des DS3231 aus.
- Sie hat Zugriff auf das Aging-Register des DS3231.

**`arc_ltc_gen`:**
- Der Generator arbeitet zeitgetrieben. Für jedes Frame wird berechnet, wann sein erstes Sample am DAC ankommt.
- Abweichungen ab 0,75 Samples korrigiert er durch Weglassen oder Wiederholen eines Samples (bei 48 kHz sind 0,75 Samples rund 16 µs).
- Das ersetzt die beim ESP32-S3 fehlende APLL-Feinabstimmung.

**`arc_engine`:**
- Jam Sync: Lock, RTC µs-genau stellen, 30 s Prüfphase mit Restversatz und Drift, Aging-Abgleich.
- Latenz-Kalibrierung per Selbst-Loopback über J2.

## Einbinden

In der `platformio.ini` des Geräts:

```ini
lib_deps =
    symlink://../../ARC-Core                              ; Entwicklung (Repo liegt daneben)
    ; https://github.com/probstj03/ARC-Core.git#v0.1.0   ; feste Version
```

Die Geräte-Firmware stellt Pins, gerätespezifische Nachrichten (`arc::net::Config::onMessage`) und
Statusfelder (`fillStatus`) bereit. Ein Beispiel ist `ARC-Slate/firmware/src/slate_link.cpp`.

## Zeitbasis und Synchronität

- **Wanduhr:** `arc::clock::nowNs()`. Ortszeit der RTC in ns seit 1970, ohne I2C-Zugriff.
- **Timecode:** `Timeline::frameAt(wallNs)`. Die Rechnung ist deterministisch: Zwei Geräte mit gejammter RTC und gleicher
  Zeitachse zeigen denselben TC, auch ohne Verbindung.
- **Geplante Befehle:** `{"type":"command","cmd":"start","at":<rtcUs>}` verankert die Zeitachse exakt auf
  `at`. Die WLAN-Latenz spielt keine Rolle.
- **Jam Sync:**
  1. Der Master läuft im Free Run, das Kabel geht J2 ↔ J2, am Slave ist `jam_free` aktiv.
  2. Nach 75 sauberen Frames stellt der Slave seine RTC per Frame-Zeitstempel.
  3. In der 30-s-Prüfphase misst er Restversatz und Drift, trimmt per Software und setzt das Aging-Register.
  4. Danach läuft er im Free Run. Bis „Kabel abgezogen“ bestätigt ist (`jam_confirm_unplug`), bleibt sein TC-Ausgang stumm.
- **Latenz:** Der Abstand von der Generator-Berechnung bis zum Signal an J2 (DMA-Puffer + DAC) wird laufend über
  den eigenen Decoder gemessen und im NVS gespeichert. Auf der ARC_AP-Platine hängen Ein- und Ausgang am
  selben Knoten.

## Protokoll v2 (Auszug)

| Richtung | Nachrichten |
|---|---|
| Gerät → Studio | `hello`, `status`/`init_sync` (mit `rtcUs`, `subframeUs`, `jam{…}`, `output{latencyUs,…}`), `event`, `pong`, `wifi_config_ack` |
| Studio → Gerät | `command` (`start`/`stop` mit `at`, `reset`, `set_tc_mode`, `set_gen_mode`, `jam_confirm_unplug`, `sync_rtc`), `ping`, `set_time`, `wifi_config` |

Die vollständige Beschreibung folgt in `ARC-Server/docs/PROTOCOL.md`.

## Tests

```bash
cd tests
pio test          # ESP32-S3 angeschlossen
```

Getestet werden die rein rechnenden Teile: Drop-Frame, Frame ↔ Zeit, Zeitachse, Userbits und Zeitformat.

## Grenzen

- Die Genauigkeit des Jam hängt am DS3231. Spezifiziert sind ±2 ppm, nach dem Aging-Abgleich typisch deutlich darunter.
  ARC-Studio erinnert an einen erneuten Jam.
- Jam → Record Run übernimmt nur den TC des Masters, nicht seine Uhrzeit. Für geplante Befehle zwischen
  Geräten zuerst einmal Jam → Free Run ausführen.
