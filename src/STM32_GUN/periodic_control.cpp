#include "periodic_control.h"

// === Variables globales ===
HardwareTimer *MyTim = nullptr;
volatile int control_counter = 0;

// Variables PID
float pid_consign = 0.0f;          // Consigne de pression (bar)
float pid_kp = 300.0f;              // Gain proportionnel (plus haut, plus brutal)
float pid_ki = 120.0f;             // Gain intégral
float pid_kd = 0.050f;              // Gain dérivé (plus haut, plus lent lors en transitoire)
bool pid_enabled = false;          // PID activé/désactivé
float pid_offset = 30.0f;          // Offset pour activation électrovanne (%)
float pid_max_output = 100.0f;     // Sortie maximale (%)
extern float delta_pressure_compensation; // compensation de perte de charge différentielle (directement sur la commande) si=4 : facteur de 1.0 à 8
extern float filter_alpha;    // Coefficient de filtrage (0.1 = filtrage fort,  0.01 encore plus fort)
extern bool auto_exhaust;
// Variables internes PID
static float error[2] = {0.0f, 0.0f};  // error[0] = actuel, error[1] = précédent
static float integral = 0.0f;          // Terme intégral
static float last_command = 0.0f;      // Dernière commande (pour filtrage)
static const float tEch = 1.0f / 200.0f;  // Période d'échantillonnage (1ms)


// Pin de lecture pression (correspond à votre #define PRESSURE_SENSOR_PIN PA0)
#define PRESSURE_SENSOR_PIN PA0
#define EXHAUST_VALVE_PORT     GPIOB
#define EXHAUST_VALVE_PIN      GPIO_PIN_3
#define RED_LED_PORT    GPIOA
#define RED_LED_PIN     GPIO_PIN_5
#define GREEN_LED_PORT  GPIOA
#define GREEN_LED_PIN   GPIO_PIN_3

// Fonction de contrôle valve (déclarée dans votre .ino)
extern void set_intake_valve_duty(uint8_t duty_percent);
extern void send_can(uint16_t pressure_value,  uint16_t can_id);

// === Implémentation des fonctions ===

void setup_periodic_control(uint32_t frequency_hz) {
  // Utiliser TIM2 avec la bibliothèque HardwareTimer d'Arduino
  MyTim = new HardwareTimer(TIM2);
  
  // Configuration de la fréquence
  MyTim->setOverflow(frequency_hz, HERTZ_FORMAT);
  
  // Attacher la fonction callback
  MyTim->attachInterrupt(periodic_control_function);
  
  // Démarrer le timer
  MyTim->resume();
  
  Serial.println("Timer périodique configuré à " + String(frequency_hz) + "Hz (HardwareTimer)");
}

void periodic_control_function(void) {
  // Incrément de la variable de test
  control_counter++;
  static float command = 0;
  static float derivative = 0;
  static bool exhaust_enabled = 0;

  static float previous_command = 0;
  static bool previous_exhaust_enabled = 0;

  uint32_t adc_sum = 0;
  for (int i = 0; i < 5; i++) {
    adc_sum += analogRead(PRESSURE_SENSOR_PIN);
  }
  uint16_t PRESSURE_SENSOR = adc_sum / 5;  // Moyenne des 5 lectures

  float pressure_bar = (PRESSURE_SENSOR / 4095.0f) * 9.99f; // 0bar - 10bar
  
  if (pressure_bar <= 0.5){ 
      HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    }
  else if (pressure_bar >=0.6){
      HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);    
  }
  if (control_counter % 20 == 0) {
    send_can(PRESSURE_SENSOR, 0x1A5);  // Appel de la fonction toutes les 100ms
    if (command != previous_command) send_can(command, 0x1A4);
    if (exhaust_enabled != previous_exhaust_enabled) send_can(exhaust_enabled, 0x1A3);
    previous_command = command;
    previous_exhaust_enabled = exhaust_enabled;
  }
  
  static float previous_pid_consign = 0;
  // Si le PID n'est pas activé, on sort
  if (pid_consign == 0.0f && previous_pid_consign !=0) {
    set_intake_valve_duty(0);
    command = 0;
  }
  previous_pid_consign = pid_consign;

  if (!pid_enabled ) {
    return;
  }
  
  // Calcul de l'erreur
  error[1] = error[0]; // Décale les erreurs pour conserver l'historique
  error[0] = pid_consign - pressure_bar; // Calcule l'erreur actuelle (consigne - mesure)
  
  // Calcul des termes PID
  integral = (error[0] * tEch + 0.9f * integral); // Accumulation pondérée des erreurs passées
  derivative = (error[0] - error[1]) / tEch; // Taux de variation actuel
  
  // Limitation anti-windup de l'intégrale
  if (integral > 500.0f) integral = 500.0f;
  if (integral < -500.0f) integral = -500.0f;
  
  command = pid_kp * error[0] + pid_ki * integral + pid_kd * derivative; // Calcul PID de base

  // plus la pression est haute, plus l'offset de la pression-consigne est grand (6bar:5.84, 5bar:4.84, 4bar:3.93, 3bar:2.96, 2bar:2.00, 1bar:1.02)
  // ceci parce que le compresseur, en amont, n'a que 8-9bar. Plus la différence entre le compresseur et la pression du canon est grande, il faudrait encore plus ouvrir la valve d'admission.
  static float compensation_factor = 1.0f; // compenser la perte de charge différentielle
  // Calcul du facteur de compensation basé sur la pression actuelle
  // Plus la pression est élevée, plus le facteur augmente
  float pressure_ratio = pressure_bar / 9.0f;  // Ratio par rapport à la pression max du compresseur (9 bar)
  compensation_factor = 1.0f + (pressure_ratio*delta_pressure_compensation);  // Facteur de 1.0 à 8

  // Application du facteur de compensation
  command *= compensation_factor;

  command += pid_offset; // Ajout de l'offset pour activation électrovanne
  
  // Filtrage passe-bas pour lisser les variations rapides
  command = last_command + filter_alpha * (command - last_command);
  last_command = command;
  
  // Limitation de la commande
  if (command < 30.0f) command = 0.0f;
  if (command > pid_max_output) command = pid_max_output;

  // Ajoutez ces variables statiques en début de fonction (après les autres static)
  static float error_samples[100] = {0.0f};     // Buffer circulaire pour les erreurs
  static int error_index = 0;                  // Index actuel dans le buffer
  static float sum_error = 0.0f;               // Somme des erreurs
  static bool error_buffer_full = false;       // Indique si le buffer est plein
  static float moyenne_error = 0.0f;           // Moyenne des erreurs

  // Calcul de la moyenne mobile des erreurs sur 20 échantillons
  sum_error -= error_samples[error_index];    // Retire l'ancienne erreur
  error_samples[error_index] = error[0];      // Ajoute la nouvelle erreur
  sum_error += error[0];                      // Ajoute à la somme
  error_index = (error_index + 1) % 100;       // Index circulaire

  if (!error_buffer_full && error_index == 0) {
    error_buffer_full = true;  // Le buffer est maintenant plein
  }
  // Calcul de la moyenne
  int nb_error_samples = error_buffer_full ? 20 : error_index;
  if (nb_error_samples > 0) {
    moyenne_error = sum_error / nb_error_samples;
  }

  if (moyenne_error<-0.50 && control_counter % 200 == 0){
    HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_RESET); //set fermé, reset ouvert    
    exhaust_enabled = true;
    send_can(exhaust_enabled, 0x1A3);
  }
  else if (moyenne_error>0.50 && control_counter % 200 == 0) {
    HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_SET); //set fermé, reset ouvert
    exhaust_enabled = false;
    send_can(exhaust_enabled, 0x1A3);
  }
  if (exhaust_enabled) {
    command = 0.0f;
  }
  // Application de la commande à la valve d'admission
  set_intake_valve_duty((uint8_t)command);
}

void set_pid_parameters(float kp, float ki, float kd, float offset) {
  pid_kp = kp;
  pid_ki = ki;
  pid_kd = kd;
  pid_offset = offset;
  Serial.println("PID: Kp=" + String(kp, 1) + " Ki=" + String(ki, 1) + " Kd=" + String(kd, 3) + " Offset=" + String(offset, 1) + "%");
}

void set_pid_enabled(bool enabled) {
  pid_enabled = enabled;
  if (!enabled) {
    // Reset des variables internes quand on désactive
    error[0] = error[1] = 0.0f;
    integral = 0.0f;
  }
  Serial.println("PID " + String(enabled ? "ACTIVÉ" : "DÉSACTIVÉ"));
}

void set_pid_consign(float consign) {
  if (consign < 0.0f) consign = 0.0f;
  if (consign > 10.0f) consign = 10.0f;
  pid_consign = consign;
  Serial.println("Consigne PID: " + String(consign, 2) + " bar");
}

int get_control_counter(void) {
  return control_counter;
}

void reset_control_counter(void) {
  control_counter = 0;
}