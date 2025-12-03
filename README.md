# Cestino intelligente per la raccolta differenziata (ESP32-CAM + TinyML)

Questo repository contiene il firmware Arduino e le librerie necessarie per il **prototipo di cestino intelligente** basato su **ESP32-CAM** e **TinyML** descritto nella tesi:

> _[Progettazione e sviluppo di un sistema intelligente low cost per la raccolta differenziata]_  
> Laurea magistrale in _[Ingegneria Informatica]_ – Università degli Studi Roma Tre  
> Autore: _[Davide Iannella]_ – Anno accademico _[2024/2025]_

L’obiettivo del sistema è **riconoscere cinque classi di rifiuto** (carta, plastica, vetro, umido, indifferenziato) direttamente **on-device**, fornendo all’utente un **feedback visivo** (display OLED) e **vocale** (DFPlayer Mini) per supportare la raccolta differenziata.

---

## Contenuto del repository

Struttura principale:

- `firmware-rifiuti-intelligenti-esp32cam.ino`  
  File principale del progetto Arduino. Contiene:
  - inizializzazione di ESP32-CAM, display OLED e DFPlayer Mini;
  - integrazione con la libreria di inferenza Edge Impulse;
  - logica di cattura immagine, esecuzione del modello FOMO e gestione del feedback visivo/vocale.

- `Rifiuti_Intelligenti_inferencing/`  
  Libreria generata automaticamente da **Edge Impulse** per il progetto TinyML:
  - modello FOMO MobileNetV2 0.35 quantizzato a 8 bit;
  - codice di inferenza ottimizzato per ESP32;
  - funzioni di utilità (gestione buffer, chiamata a `run_classifier()` ecc.).

- `Adafruit_GFX_Library/`  
  Libreria grafica di Adafruit utilizzata per la gestione di testo e primitive grafiche sul display OLED.

- `Adafruit_SSD1306/`  
  Driver per il display OLED 128×64 basato su controller SSD1306 (interfaccia I²C).

- `Adafruit_BusIO/`  
  Libreria di supporto richiesta dalle librerie Adafruit (GFX/SSD1306).

- `DFRobotDFPlayerMini/`  
  Libreria per il controllo del modulo **DFPlayer Mini** via seriale (riproduzione tracce MP3, volume, gestione errori).

- `ArduinoBLE/`  
  Libreria Arduino per BLE. **Non è utilizzata nel firmware attuale**, ma è stata mantenuta nel repository per eventuali estensioni future (es. controllo via smartphone). Può essere rimossa se non necessaria.

File/Documenti:

- `hardware.md` – riassunto dei collegamenti hardware principali.
- `model-notes.md` – informazioni sul progetto Edge Impulse e sul modello FOMO.
- `LICENSE` – (opzionale) licenza del progetto.
- `docs/` – schemi elettrici, schema a blocchi e foto del prototipo.

---

## Requisiti hardware

Per replicare il prototipo sono necessari:

- Modulo **ESP32-CAM AI Thinker** (ESP32 + camera OV2640).
- Display **OLED 0,96" 128×64** I²C (controller SSD1306).
- Modulo **DFPlayer Mini** + **altoparlante 3 W / 4 Ω**.
- Scheda **microSD** (es. 16–32 GB, FAT32) con i file audio:
  - `0001_umido.mp3`
  - `0002_carta.mp3`
  - `0003_vetro.mp3`
  - `0004_plastica.mp3`
  - `0005_indifferenziato.mp3`
- Barre LED opaline a 5 V per l’illuminazione interna del box.
- Box in plastica (circa 23×23×23 cm) che contiene camera, illuminazione e area di conferimento.
- Alimentazione 5 V tramite porta USB (es. alimentatore da smartphone o power bank).

I collegamenti elettrici principali sono riassunti in `hardware.md`.

---

## Requisiti software

- **Arduino IDE** (versione recente).
- **Core ESP32** installato tramite Board Manager  
  (scheda da selezionare: `AI Thinker ESP32-CAM`).
- Librerie Arduino (se non incluse direttamente nel repository):
  - `Adafruit_GFX_Library`
  - `Adafruit_SSD1306`
  - `Adafruit_BusIO`
  - `DFRobotDFPlayerMini`
  - (opzionale) `ArduinoBLE`
- Libreria Edge Impulse:
  - `Rifiuti_Intelligenti_inferencing` (contenuta nel repository; da copiare nella cartella `libraries/` dell’IDE se necessario).
