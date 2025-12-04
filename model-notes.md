# Note sul modello TinyML e sul progetto Edge Impulse

Questo documento riassume le caratteristiche principali del modello TinyML utilizzato nel prototipo e il flusso di lavoro su **Edge Impulse**.

---

## 1. Obiettivo del modello

Il modello è progettato per:

- riconoscere **cinque classi di rifiuto**:
  - `carta`
  - `plastica`
  - `vetro`
  - `umido`
  - `indifferenziato`
- operare **on–device** su **ESP32-CAM**, con:
  - memoria limitata (RAM ≈ 512 kB disponibili per il modello),
  - alimentazione a 5 V,
  - tempo di inferenza intorno a **1 secondo** per frame.

---

## 2. Dataset e pre–processing (riassunto)

- Dataset costruito **ad hoc** a partire da immagini di rifiuti acquisite con ESP32-CAM nelle condizioni del prototipo (box, illuminazione LED, distanza ≈ 20 cm).
- Pre–processing eseguito in Edge Impulse:
  - blocco **Image**;
  - ridimensionamento a **96 × 96** pixel;
  - conversione in **scala di grigio**;
  - vettore di input: 9216 caratteristiche (1 valore per pixel).

Per l’addestramento del modello FOMO sono stati utilizzati:

- ~**500 immagini** annotate per l’object detection (circa 100 per classe).

---

## 3. Architettura scelta (modello finale)

L’architettura finale deployata nella libreria `Rifiuti_Intelligenti_inferencing` è:

- **FOMO (Faster Objects, More Objects)** – modello di object detection ottimizzato per microcontrollori.
- **Backbone**: **MobileNetV2** con:
  - fattore di larghezza (**width multiplier**) `α = 0.35`;
  - convoluzioni depthwise separable;
  - numero di parametri ridotto rispetto alla MobileNetV2 “piena”.
- **Input**: immagini 96 × 96 in scala di grigio.
- **Output**:
  - mappa di celle (griglia) con, per ciascuna cella:
    - probabilità di appartenenza alle classi (`carta`, `plastica`, `vetro`, `umido`, `indifferenziato`, `background`);
    - bounding box approssimativa.

---

## 4. Configurazione di training (esperimento finale)

Esperimento selezionato per il deployment:

- Modello: **FOMO + MobileNetV2 0.35**
- **Learning rate**: `0.001` (ridotto rispetto a 0.01 per maggiore stabilità).
- **Epoche**: 60
- **Ottimizzatore**: (impostazione di default Edge Impulse, tipicamente Adam/AdamW)
- **Data augmentation**:
  - rotazioni leggere;
  - piccoli shift in orizzontale/verticale;
  - variazioni di luminosità.
- **Split dei dati**:
  - ~80% training;
  - ~20% validation (gestito automaticamente da Edge Impulse).

---

## 5. Prestazioni (validation set Edge Impulse)

Valori indicativi sul validation set (modello finale):

- **F1–score medio** (classi non–background): ≈ **98%**
- **Precisione media**: ≈ **0.99**
- **Recall media**: ≈ **0.98**
- Prestazioni particolarmente buone per:
  - `carta`
  - `umido`
  - `indifferenziato`.
- Classe più critica:
  - `vetro` (in particolare vetro chiaro, a causa di riflessi e trasparenze).

Queste prestazioni sono coerenti con i risultati qualitativi osservati nelle prove sul prototipo.

---

## 6. Ottimizzazione TinyML e deployment

Per l’esecuzione su ESP32-CAM sono state applicate le seguenti ottimizzazioni:

- **Quantizzazione a 8 bit** (`Quantized int8`)
  - riduce dimensione del modello in flash;
  - riduce l’uso di RAM;
  - consente inferenza con sola aritmetica intera.
- Utilizzo del **compilatore EON** di Edge Impulse per generare codice ottimizzato per ESP32.

---

## 7. Libreria di inferenza (`Rifiuti_Intelligenti_inferencing`)

La cartella `Rifiuti_Intelligenti_inferencing/` contiene:

- Implementazione del modello FOMO quantizzato.
- API C/C++ per l’inferenza, tra cui:
  - `run_classifier()` – funzione principale chiamata nel firmware.
  - Strutture dati per gestire:
    - segnale in ingresso (`ei::signal_t`);
    - risultati (`ei_impulse_result_t`), comprese le bounding box.

Nel file `firmware-rifiuti-intelligenti-esp32cam.ino` il flusso è:

1. Acquisizione immagine da ESP32-CAM.
2. Conversione e adattamento al formato richiesto da Edge Impulse.
3. Chiamata a `run_classifier()`.
4. Ricerca della classe con probabilità massima tra le bounding box rilevate.
5. Aggiornamento del display OLED e avvio della traccia audio corrispondente sul DFPlayer.

---

## 8. Possibili estensioni del modello

Alcune direzioni di sviluppo previste nella tesi:

- **Dataset ampliato**:
  - più oggetti per classe, materiali trasparenti/critici, più condizioni di illuminazione.
- **Rifiuti multipli**:
  - addestramento esplicito su immagini con più oggetti contemporaneamente.
- **Materiali critici (vetro chiaro, plastica trasparente)**:
  - data augmentation mirata;
  - eventuali modifiche alla pipeline di pre–processing.
- **Aggiornamenti incrementali**:
  - futura possibilità di aggiornare il modello con nuovi dati raccolti in campo, mantenendo il deployment on–device.

