/*
 * ESP32-CAM object detection code + OLED + DFPlayer
 * Progetto di classificazione rifiuti con modello TinyML (Edge Impulse)
 */

#include <Rifiuti_Intelligenti_inferencing.h>  // Header generato da Edge Impulse
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// Selezione modello di camera (qui: AI-Thinker). I pin vengono definiti piu sotto.


 // #define CAMERA_MODEL_ESP_EYE  // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 4
#define SIOD_GPIO_NUM 18
#define SIOC_GPIO_NUM 23
#define Y9_GPIO_NUM 36
#define Y8_GPIO_NUM 37
#define Y7_GPIO_NUM 38
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 35
#define Y4_GPIO_NUM 14
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM 27
#define PCLK_GPIO_NUM 25

#elif defined(CAMERA_MODEL_AI_THINKER)

// Pinout specifico per il modulo ESP32-CAM AI-Thinker
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

/* Costanti per il buffer immagine usato da Edge Impulse -------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobotDFPlayerMini.h> 

// -------------------------------
// Configurazione display OLED I2C
// -------------------------------
#define OLED_SDA 2    
#define OLED_SCL 15   

// Display defines
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// LED flash della ESP32-CAM (non usato per la luce del box in questo sketch)
#define FLASH_LED_PIN 4

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ----------------------------------------------------
// Configurazione DFPlayer Mini (audio feedback vocale)
// ----------------------------------------------------
#define DFPLAYER_RX_PIN 13   // ESP32 riceve da TX del DFPlayer
#define DFPLAYER_TX_PIN 14   // ESP32 trasmette a RX del DFPlayer

// UART1 (non UART2) per evitare conflitti con PSRAM / camera
HardwareSerial FPSerial(1);

DFRobotDFPlayerMini myDFPlayer;
bool dfplayer_ok = false;
char last_label_played[16] = "";

/* Variabili per la camera / Edge Impulse ----------------------------------- */
static bool debug_nn = false;  // Se true, stampa info di debug del modello
static bool is_initialised = false;
uint8_t *snapshot_buf;         // Puntatore al frame acquisito dalla camera

// Configurazione della camera per esp_camera_init()
static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,
  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,
  // XCLK 20MHz (clock per la OV2640)
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,  // Formato nativo usato dalla camera
  .frame_size = FRAMESIZE_QVGA,    // 320x240, adatto per Edge Impulse
  .jpeg_quality = 12,              // 0-63: numero piu basso = qualita piu alta
  .fb_count = 1,                   // 1 frame buffer in PSRAM
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Prototipi delle funzioni di supporto per la camera ----------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

/**
 * @brief  Setup Arduino: inizializza seriale, OLED, DFPlayer e camera
 */
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  Serial.println();
  Serial.println(F("ESP32-CAM TinyML + DFPlayer + OLED"));

  // ----- OLED -----
  Wire.begin(OLED_SDA, OLED_SCL);  // SDA=2, SCL=15

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("ERRORE OLED"));
    // Non blocchiamo il programma: il sistema puo funzionare anche senza display
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Init OLED OK");
  display.display();

  // ----- DFPLAYER (inizializzato prima della camera) -----
  display.setCursor(0, 16);
  display.print("Init DFPlayer...");
  display.display();

  Serial.println(F("Prima di FPSerial.begin"));
  FPSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  Serial.println(F("Dopo FPSerial.begin"));

  if (!myDFPlayer.begin(FPSerial)) {
    Serial.println(F("DFPlayer non inizializzato, controlla cablaggi/SD."));
    dfplayer_ok = false;

    display.setCursor(0, 32);
    display.print("DF NOK");
    display.display();
  } else {
    dfplayer_ok = true;
    Serial.println(F("DFPlayer Mini online."));
    myDFPlayer.volume(30);        // volume massimo 

    display.setCursor(0, 32);
    display.print("DF OK");
    display.display();
  }

  // ----- CAMERA (dopo DFPlayer) -----
  if (ei_camera_init() == false) {
    ei_printf("Failed to initialize Camera!\r\n");
    display.setCursor(0, 48);
    display.print("Cam ERR");
    display.display();
  } else {
    ei_printf("Camera initialized\r\n");
  }

  // Messaggio iniziale prima dell'inferenza continua
  ei_printf("\nStarting continuous inference in 2 seconds...\n");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Starting\ninference");
  display.display();
  ei_sleep(2000);
  display.clearDisplay();
}

/**
 * @brief  Riproduce la traccia MP3 corretta in base all'etichetta EI
 *         (mappa le label del modello in numeri di traccia DFPlayer).
 */
void playAudioForLabel(const char *label) {
  if (!dfplayer_ok) return;

  // Debounce: evita di parlare piu di una volta ogni 1500 ms
  static unsigned long last_play_ms = 0;
  unsigned long now = millis();
  if (now - last_play_ms < 1500) {
    return;
  }

  uint16_t track = 0;

  if (strcmp(label, "umido") == 0) {
    track = 1;    // 0001.mp3
  }
  else if (strcmp(label, "carta") == 0) {
    track = 2;    // 0002.mp3
  }
  else if (strcmp(label, "plastica") == 0) {
    track = 3;    // 0003.mp3
  }
  else if (strcmp(label, "vetro") == 0) {
    track = 4;    // 0004.mp3
  }
  else if (strcmp(label, "indifferenziato") == 0) {
    track = 5;    // 0005.mp3
  } else {
    // Etichetta non gestita: non riprodurre audio
    return;
  }

  myDFPlayer.play(track);

  // Memorizza ultima label pronunciata (eventuale uso futuro)
  strncpy(last_label_played, label, sizeof(last_label_played) - 1);
  last_label_played[sizeof(last_label_played) - 1] = '\0';
  last_play_ms = now;
}

/* ==================================================================== */
/**
 * @brief  Ciclo principale:
 *         - acquisisce un'immagine
 *         - esegue il modello TinyML (Edge Impulse)
 *         - mostra etichetta su OLED
 *         - riproduce audio corrispondente via DFPlayer
 *
 * @param[in]  debug  (non usato qui) se true stampa info extra
 */
void loop() {
  display.clearDisplay();

  // Piccola attesa; permette anche di interrompere il task se necessario
  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }

  // Allocazione dinamica del buffer per l'immagine catturata
  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
  if (snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  // Costruzione del "signal" Edge Impulse a partire dai pixel dell'immagine
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  // Acquisizione dell'immagine dalla camera
  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    return;
  }

  // Esecuzione del classificatore Edge Impulse
  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // Stampa dei tempi di esecuzione
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
 
  // Modalita object detection: si sceglie UNA sola etichetta "migliore" per frame
  // (quella con confidenza massima tra le bounding box trovate).

  bool bb_found = false;
  const char *best_label = nullptr;
  float best_value = 0.0f;

  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
    auto bb = result.bounding_boxes[ix];
    if (bb.value == 0) {
      continue;
    }

    bb_found = true;

    // Tiene traccia della bbox con confidenza massima
    if (bb.value > best_value) {
      best_value = bb.value;
      best_label = bb.label;
    }

    // Log su seriale
    ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n",
              bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
  }

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  if (!bb_found || best_label == nullptr) {
    ei_printf("    No objects found\n");
    // Messaggio su due righe per non uscire dai 128 px del display
    display.setCursor(0, 16);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    display.println("No objects");
    display.setCursor(0, 32);
    display.println("found");
    display.display();

  } else {
    // Visualizza SOLO l'etichetta migliore + percentuale di confidenza
    display.setCursor(0, 16);
    display.print(best_label);
    display.print("-");
    display.print(int(best_value * 100));
    display.print("%");
    display.display();

    // Pronuncia la stessa etichetta mostrata sul display
    playAudioForLabel(best_label);
  }

  // Secondo controllo "no objects" (ridondante ma innocuo per la logica)
  if (!bb_found) {
    ei_printf("    No objects found\n");
    display.setCursor(0, 16);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    display.println("No objects");
    display.setCursor(0, 32);
    display.println("found");
    display.display();
  }

#else
  // Modalita classificazione standard (non object detection)
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label,
              result.classification[ix].value);
  }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

  // Liberiamo il buffer immagine per non saturare la PSRAM
  free(snapshot_buf);
}

/**
 * @brief   Inizializza il sensore di immagine e la pipeline della camera.
 *
 * @retval  false se l'inizializzazione fallisce.
 */
bool ei_camera_init(void) {
  if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
   pinMode(13, INPUT_PULLUP);
   pinMode(14, INPUT_PULLUP);
#endif

  // Inizializzazione della camera tramite driver Espressif
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  // Piccole correzioni di immagine se necessario (dipendono dal sensore)
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);       // flip verticale
    s->set_brightness(s, 1);  // aumenta leggermente la luminosita
    s->set_saturation(s, 0);  // saturazione neutra
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  s->set_awb_gain(s, 1);
#endif

  is_initialised = true;
  return true;
}

/**
 * @brief  Deinizializza la camera (non usato nel loop, ma utile come utility).
 */
void ei_camera_deinit(void) {
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ei_printf("Camera deinit failed\n");
    return;
  }
  is_initialised = false;
  return;
}

/**
 * @brief  Cattura un'immagine dalla camera, la converte in RGB888 e la ridimensiona.
 *
 * @param[in]  img_width   larghezza di output
 * @param[in]  img_height  altezza di output
 * @param[in]  out_buf     buffer destinazione (puo coincidere con snapshot_buf)
 *
 * @retval     false in caso di errore di inizializzazione o conversione
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  bool do_resize = false;
  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  // Converte da JPEG della camera a RGB888 nel buffer snapshot_buf
  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);
  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  // Se le dimensioni richieste non coincidono, si effettua un resize
  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
      || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    do_resize = true;
  }

  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height);
  }
  return true;
}

/**
 * @brief  Funzione di callback che Edge Impulse usa per leggere i pixel dal buffer.
 *
 *         Converte il buffer RGB888 in valori interi (uno per pixel) nel formato
 *         atteso dal modello (riordina anche da BGR a RGB).
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  // Buffer RGB888 gia disponibile: calcoliamo l'indice del pixel di partenza
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    // Swap BGR -> RGB (workaround per issue Espressif)
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) +
                          (snapshot_buf[pixel_ix + 1] << 8) +
                           snapshot_buf[pixel_ix];
    // Pixel successivo
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
