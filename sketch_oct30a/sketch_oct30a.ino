#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define ADS7830_ADDR 0x4B

LiquidCrystal_I2C lcd(0x27, 16, 2);
TaskHandle_t blinkerHandle = nullptr;
//SDA = 21
//SCL = 22

struct LedClignote {
  int pin;
  int on_ms;
  int off_ms;
  const char* name;
};

// ---------- Pins ----------
const int LED_BLINKER_PIN = 12;
const int LEVER_BLINKER_PIN = 34;
const int PHARE_PIN = 23;
const int BUZZER_PIN = 19;
const int MOTOR_IN3_PIN = 14;
const int MOTOR_IN4_PIN = 27;

// ---------- Phares / klaxon ----------
const int SEUIL_NUIT = 110;
const int SEUIL_JOUR = 170;
const int KLAXON_FREQ = 440;

LedClignote blinker = { LED_BLINKER_PIN, 500, 250, "t1" };

// ---------- Joystick ----------
const int JOYSTICK_CENTRE = 126;
const int ZONE_MORTE = 10;  // joystick jitter around the centre

// ---------- Vehicule (unite : Kerbal/s) ----------
const float VITESSE_MIN = -50.0f;
const float VITESSE_MAX = 100.0f;
const float ACCEL_AVANT_MAX = 5.0f;    // Kerbal/s^2 at full forward deflection
const float ACCEL_ARRIERE_MAX = 3.0f;  // Kerbal/s^2 at full backward deflection
const float DECEL_NATURELLE = 1.0f;    // Kerbal/s^2 toward 0, always applied
const int PWM_MOTEUR_FREQ = 1000;
const int PWM_MOTEUR_RES = 8;  // duty 0..255
const int MOTEUR_DUTY_MIN = 120;  // minimum duty that makes the motor actually turn (tune this!)

float vitesse_f = 0.0f;  // precise speed, -50 .. 100

// ---------- LCD custom characters (5x8 pixels each) ----------
byte CHAR_AVANT[8] = { B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00000 };    // up arrow
byte CHAR_ARRIERE[8] = { B00100, B00100, B00100, B00100, B10101, B01110, B00100, B00000 };  // down arrow
byte CHAR_AMPOULE[8] = { B01110, B10001, B10001, B10001, B01010, B01110, B01110, B00100 };  // bulb

const int SLOT_AVANT = 0;
const int SLOT_ARRIERE = 1;
const int SLOT_AMPOULE = 2;

void afficher_lcd(int vit, bool clignotant) {
  static int dernier_abs = -1;
  static int derniere_dir = -2;
  static int dernier_cligno = -1;

  int valeur = abs(vit);
  int dir = (vit > 0) ? 1 : (vit < 0) ? -1 : 0;

  // Only touch the LCD when something changed (I2C is slow)
  if (valeur != dernier_abs) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%-3d", valeur);  // padded to 3 chars, erases leftovers
    lcd.setCursor(9, 0);
    lcd.print(buf);
    dernier_abs = valeur;
  }

  if (dir != derniere_dir) {
    lcd.setCursor(15, 0);  // top right
    if (dir > 0) lcd.write((uint8_t)SLOT_AVANT);
    else if (dir < 0) lcd.write((uint8_t)SLOT_ARRIERE);
    else lcd.print(" ");
    derniere_dir = dir;
  }

  if ((int)clignotant != dernier_cligno) {
    lcd.setCursor(15, 1);  // bottom right
    if (clignotant) lcd.write((uint8_t)SLOT_AMPOULE);
    else lcd.print(" ");
    dernier_cligno = (int)clignotant;
  }
}

void task_clignote(void* userdata) {
  LedClignote* led = (LedClignote*)userdata;

  for (;;) {
    digitalWrite(led->pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(led->on_ms));
    digitalWrite(led->pin, LOW);
    vTaskDelay(pdMS_TO_TICKS(led->off_ms));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LEVER_BLINKER_PIN, INPUT);
  pinMode(PHARE_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.createChar(SLOT_AVANT, CHAR_AVANT);
  lcd.createChar(SLOT_ARRIERE, CHAR_ARRIERE);
  lcd.createChar(SLOT_AMPOULE, CHAR_AMPOULE);
  lcd.setCursor(0, 0);
  lcd.print("Kerbal/s");

  pinMode(blinker.pin, OUTPUT);

  // Motor PWM first, buzzer after: each group then gets its own LEDC timer,
  // so the buzzer's ledcWriteTone() doesn't change the motor's PWM frequency.
  ledcAttach(MOTOR_IN3_PIN, PWM_MOTEUR_FREQ, PWM_MOTEUR_RES);
  ledcAttach(MOTOR_IN4_PIN, PWM_MOTEUR_FREQ, PWM_MOTEUR_RES);
  ledcWrite(MOTOR_IN3_PIN, 0);
  ledcWrite(MOTOR_IN4_PIN, 0);

  ledcAttach(BUZZER_PIN, 2000, 8);
}

// Valeurs de sorties : -1 pour err, sinon entre 0 et 255.
// addr = adresse du module adc
// int channel = le canal de l'adc. A0 = 0, A1 = 1, ...
int ads7830_read_selected_channel(int addr, int channel) {
  const int read_command_base = 0x84;

  // Transmet la commande de sélection de canal
  Wire.beginTransmission(addr);
  {
    // 3 bits dans l'octet de commande sont utilisés pour indiquer le canal de lecture
    // mais le mapping est un peu bizarre :
    // En considérant le nibble suivant : MSB C2 C1 C0
    // C2 = 1
    // C1 = 4
    // C0 = 2
    // Le canal 5 devient : MSB 1 1 0
    // Le canal 3 devient : MSB 1 0 1
    // Le canal 6 devient : MSB 0 1 1
    unsigned int arg_channel = (((channel << 2) | (channel >> 1)) & 0x07) << 4;
    unsigned char cmd = (unsigned char)((read_command_base | arg_channel) & 0xFF);
    Wire.write(cmd);
  }
  Wire.endTransmission();

  // Lit la réponse.
  Wire.requestFrom(addr, 1);
  if (!Wire.available()) {
    return -1;
  }

  return Wire.read();
}

int readLeverBlinker() {
  return digitalRead(LEVER_BLINKER_PIN);
}

int readADC(int channel) {
  return ads7830_read_selected_channel(ADS7830_ADDR, channel);
}

// Joystick -> acceleration in Kerbal/s^2 (positive = forward, negative = backward).
// The further the stick is pushed, the stronger the acceleration, up to the max.
float acceleration_joystick(int joystick) {
  if (joystick < 0) return 0.0f;  // I2C error: no input

  const int seuil_avant = JOYSTICK_CENTRE + ZONE_MORTE;
  const int seuil_arriere = JOYSTICK_CENTRE - ZONE_MORTE;

  if (joystick > seuil_avant) {
    float f = (float)(joystick - seuil_avant) / (float)(255 - seuil_avant);
    return constrain(f, 0.0f, 1.0f) * ACCEL_AVANT_MAX;
  }
  if (joystick < seuil_arriere) {
    float f = (float)(seuil_arriere - joystick) / (float)seuil_arriere;
    return -constrain(f, 0.0f, 1.0f) * ACCEL_ARRIERE_MAX;
  }
  return 0.0f;
}

// dt = seconds since the last update
void mettre_a_jour_vitesse(int joystick, float dt) {
  vitesse_f += acceleration_joystick(joystick) * dt;

  // Natural deceleration toward 0 (never overshoots past 0)
  float pas = DECEL_NATURELLE * dt;
  if (vitesse_f > pas) vitesse_f -= pas;
  else if (vitesse_f < -pas) vitesse_f += pas;
  else vitesse_f = 0.0f;

  vitesse_f = constrain(vitesse_f, VITESSE_MIN, VITESSE_MAX);
}

// vit: -50 .. 100. Any non-zero speed maps to a duty between MOTEUR_DUTY_MIN
// (just enough to get the motor turning) and 255 (|speed| = 100).
void appliquer_moteur(int vit) {
  int duty = 0;
  if (vit != 0) {
    duty = map(abs(vit), 1, (int)VITESSE_MAX, MOTEUR_DUTY_MIN, 255);
  }
  if (vit > 0) {  // forward: IN3 = PWM, IN4 = L
    ledcWrite(MOTOR_IN3_PIN, duty);
    ledcWrite(MOTOR_IN4_PIN, 0);
  } else if (vit < 0) {  // backward: IN3 = L, IN4 = PWM
    ledcWrite(MOTOR_IN3_PIN, 0);
    ledcWrite(MOTOR_IN4_PIN, duty);
  } else {  // neutral: no current
    ledcWrite(MOTOR_IN3_PIN, 0);
    ledcWrite(MOTOR_IN4_PIN, 0);
  }
}

void loop() {
  static int last_lever_reading = -1;
  static bool phares_allumes = false;
  static bool klaxon_actif = false;
  static int derniere_dir = 0;
  static unsigned long dernier_tick = millis();

  unsigned long maintenant = millis();
  float dt = (maintenant - dernier_tick) / 1000.0f;
  dernier_tick = maintenant;

  int joystick_x = readADC(1);
  int joystick_piton = readADC(2);
  int luminosite = readADC(3);

  bool piton_enfonce = joystick_piton < 10 && joystick_piton >= 0;

  // --- Klaxon ---
  if (piton_enfonce != klaxon_actif) {
    klaxon_actif = piton_enfonce;
    ledcWriteTone(BUZZER_PIN, klaxon_actif ? KLAXON_FREQ : 0);
  }

  // --- Phares (hysteresis) ---
  if (luminosite >= 0) {
    if (!phares_allumes && luminosite < SEUIL_NUIT) {
      phares_allumes = true;
    } else if (phares_allumes && luminosite > SEUIL_JOUR) {
      phares_allumes = false;
    }
    digitalWrite(PHARE_PIN, phares_allumes ? HIGH : LOW);
  }

  // --- Clignotant ---
  int newReading = readLeverBlinker();
  if (newReading != last_lever_reading) {
    if (newReading == HIGH) {
      if (blinkerHandle == nullptr) {
        xTaskCreate(task_clignote, blinker.name, 2048, &blinker,
                    tskIDLE_PRIORITY + 1, &blinkerHandle);
      }
    } else {
      if (blinkerHandle != nullptr) {
        vTaskDelete(blinkerHandle);
        blinkerHandle = nullptr;
      }
      digitalWrite(blinker.pin, LOW);
    }
    last_lever_reading = newReading;
  }

  // --- Vitesse et moteur ---
  // To accelerate with the Y axis instead, pass readADC(0) here.
  mettre_a_jour_vitesse(joystick_x, dt);
  int vitesse = lround(vitesse_f);
  appliquer_moteur(vitesse);

  // Console message when the direction changes to forward or backward
  int dir = (vitesse > 0) ? 1 : (vitesse < 0) ? -1 : 0;
  if (dir != derniere_dir) {
    if (dir > 0) Serial.println("Forward");
    else if (dir < 0) Serial.println("Backward");
    derniere_dir = dir;
  }

  afficher_lcd(vitesse, blinkerHandle != nullptr);
  delay(20);  // ~50 updates/s, well above the 10/s minimum
}
