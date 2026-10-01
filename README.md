# ESP-Bewaesserung-Blynk
Bilder und Dateien für mein ESP32-Blynk-Bewässerungsprojekt


## Firmware

Aktueller Arduino-Sketch für das Freenove ESP32-S3-WROOM Board:
`firmware/ESP_Bewaesserung_Blynk/ESP_Bewaesserung_Blynk.ino`.

Vor dem Kompilieren `secrets.example.h` im selben Ordner nach `secrets.h`
kopieren und die eigenen WLAN-, Blynk- und Telegram-Daten lokal eintragen.
`secrets.h` wird nicht in Git gespeichert. Zugangsdaten niemals committen.

Benötigt werden das ESP32-Arduino-Core und die Bibliotheken Blynk, OneWire,
DallasTemperature, Adafruit MAX1704X und Adafruit NeoPixel.

Dieser Import wurde noch nicht kompiliert oder auf Hardware getestet.
