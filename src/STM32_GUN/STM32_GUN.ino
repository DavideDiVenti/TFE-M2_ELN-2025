#include "STM32_CAN.h"
#include "stm32f3xx_hal.h"
#include "periodic_control.h"


// === Définition des broches OUTPUT ===
#define RED_LED_PORT    GPIOA
#define RED_LED_PIN     GPIO_PIN_5

#define YELLOW_LED_PORT GPIOA
#define YELLOW_LED_PIN  GPIO_PIN_4

#define GREEN_LED_PORT  GPIOA
#define GREEN_LED_PIN   GPIO_PIN_3

#define BUZZER_PORT     GPIOB
#define BUZZER_PIN      GPIO_PIN_1 // couper PB7, relier à PB6

#define ELECTROMAGNET_PORT     GPIOA
#define ELECTROMAGNET_PIN      GPIO_PIN_10
// -- to do
#define FIRING_VALVE_PORT     GPIOA
#define FIRING_VALVE_PIN      GPIO_PIN_1

#define EXHAUST_VALVE_PORT     GPIOB
#define EXHAUST_VALVE_PIN      GPIO_PIN_3

#define TRIGGER_PORT     GPIOB
#define TRIGGER_PIN      GPIO_PIN_0

// === Définition des broches INPUT ===
#define BUTTON_EMERGENCY_STOP_PORT GPIOA
#define BUTTON_EMERGENCY_STOP_PIN  GPIO_PIN_7

#define INT_EMERGENCY_STOP_PORT    GPIOB
#define INT_EMERGENCY_STOP_PIN     GPIO_PIN_5

#define END_STOP_PORT    GPIOA
#define END_STOP_PIN     GPIO_PIN_9
// -- to do
#define EXHAUST_BUTTON_PORT    GPIOA   // mettre un pull up
#define EXHAUST_BUTTON_PIN     GPIO_PIN_6

// === ADC ==
#define PRESSURE_SENSOR_PIN PA0

// === PWM ===
#define PWM_TTL_INTAKE_VALVE_PORT    GPIOB   
#define PWM_TTL_INTAKE_VALVE_PIN     GPIO_PIN_4
#define PWM_PWR_INTAKE_VALVE_PORT    GPIOA   
#define PWM_PWR_INTAKE_VALVE_PIN     GPIO_PIN_8

// === Structure GPIO ===
GPIO_InitTypeDef GPIO_InitStruct;
TIM_HandleTypeDef htim1;  // Pour PA8 (TIM1_CH1)
TIM_HandleTypeDef htim3;  // Pour PB4 (TIM3_CH1)
TIM_OC_InitTypeDef sConfigOC;


STM32_CAN Can( CAN1, DEF );  //Use PA11/12 pins for CAN1.
static CAN_message_t CAN_TX_msg;
// -- CAN GLOBAL --
float P=60;
float I=40;
float D=0.05;
bool ttlPWMValve = 0;
float pressureConsign = 0; 
float pressure_bar = 0;
//pulseTime : local
//intake : local
//exhaust : local
//firing : local
//trigger : local
//TriggerPulseTime : local (before hit) = 10
bool auto_exhaust = 1;
float delta_pressure_compensation = 4.0f; // compensation de perte de charge différentielle (directement sur la commande) si=4 : facteur de 1.0 à 8
float filter_alpha = 0.01f;    // Coefficient de filtrage (0.1 = filtrage fort,  0.01 encore plus fort)


static bool myClock = 0;
static bool myPreviousClock = 0;
static int myClockPeriodMs = 200;

bool CAN_msg_received = false;

void setup_pwm() {
  // Activer les horloges pour les GPIOs et les timers
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  
  // Configuration de TIM1 pour PA8 (TIM1_CH1)
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 159;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim1);
  
  // Configuration de TIM3 pour PB4 (TIM3_CH1)
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim3);
  
  // Configuration du canal pour PA8 (TIM1_CH1)
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;  // Nécessaire pour TIM1
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;  // Nécessaire pour TIM1
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);  // PA8 est sur TIM1_CH1
  
  // Configuration du canal pour PB4 (TIM3_CH1)
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);  // PB4 est sur TIM3_CH1
  
  // Configuration de PA8 en fonction alternative (AF6 pour TIM1_CH1)
  GPIO_InitStruct.Pin = PWM_PWR_INTAKE_VALVE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;  // AF6 pour TIM1_CH1 sur PA8
  HAL_GPIO_Init(PWM_PWR_INTAKE_VALVE_PORT, &GPIO_InitStruct);
  
  // Configuration de PB4 en fonction alternative (AF2 pour TIM3_CH1)
  GPIO_InitStruct.Pin = PWM_TTL_INTAKE_VALVE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;  // AF2 pour TIM3_CH1 sur PB4
  HAL_GPIO_Init(PWM_TTL_INTAKE_VALVE_PORT, &GPIO_InitStruct);
  
  // Activer les sorties du timer 1 (nécessaire pour TIM1)
  __HAL_TIM_MOE_ENABLE(&htim1);
  
  // Démarrage des PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PA8
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PB4
}

void set_intake_pwr_valve_duty(uint8_t duty_percent) {
  if (duty_percent > 100) duty_percent = 100;
  
  uint32_t pulse = (uint32_t)(((htim1.Init.Period + 1) * duty_percent) / 100);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);  // PA8 est sur TIM1_CH1
}

void set_intake_ttl_valve_duty(uint8_t duty_percent) {
  if (duty_percent > 100) duty_percent = 100;
  
  uint32_t pulse = (uint32_t)(((htim3.Init.Period + 1) * duty_percent) / 100);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);  // PB4 est sur TIM3_CH1
}

void set_intake_valve_duty(uint8_t duty_percent){
  if (ttlPWMValve == 1){
    set_intake_ttl_valve_duty(duty_percent);
    set_intake_pwr_valve_duty(100);
  }
  else {
    set_intake_ttl_valve_duty(0);
    set_intake_pwr_valve_duty(duty_percent);
  }
}

void executeFiringSequence(uint32_t pulseTime, uint8_t triggerTime) {
    uint32_t startTime = HAL_GetTick();
    uint32_t triggerStartTime = startTime;
    uint32_t triggerEndTime = startTime + triggerTime;
    uint32_t firingEndTime = startTime + pulseTime;
    bool triggerDone = false;
    
    Serial.print(String(pulseTime) + "ms");   
    HAL_GPIO_WritePin(FIRING_VALVE_PORT, FIRING_VALVE_PIN, GPIO_PIN_SET); // Activer d'abord le tir  
    HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET); // Activer le trigger
    
    while (HAL_GetTick() < firingEndTime) {    // Boucle active qui gère les timings sans bloquer
        if (!triggerDone && HAL_GetTick() >= triggerEndTime) { // Vérifier si c'est le moment de désactiver le trigger
            HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
            triggerDone = true;
        }
    }
      
    if (!triggerDone) { // Assurer que le trigger est bien désactivé (au cas où)
        HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
    }  
    HAL_GPIO_WritePin(FIRING_VALVE_PORT, FIRING_VALVE_PIN, GPIO_PIN_RESET); // Désactiver le tir à la fin
    if (auto_exhaust) {
      HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_RESET); // Echapement automatique
      send_can(1, 0x1A3);
    }    
}

// Fonction de réception complète adaptée pour les messages de 4 octets
void CAN_Msg_Checking() {
  static CAN_message_t CAN_RX_msg;
  if (Can.read(CAN_RX_msg)) {
    char buf[64];
    sprintf(buf, "Channel:%d ", CAN_RX_msg.bus);
    Serial.print(buf);
    
    if (CAN_RX_msg.flags.extended == false) { 
      Serial.print("Standard ID:");
    } else {
      Serial.print("Extended ID:");
    }
    
    sprintf(buf, "%X DLC: %d ", CAN_RX_msg.id, CAN_RX_msg.len);
    Serial.print(buf);
    
    if (CAN_RX_msg.flags.remote == false) {
      Serial.print("buf: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        sprintf(buf, "0x%X ", CAN_RX_msg.buf[i]);
        Serial.print(buf);
      }
      Serial.println();
      
      // === LECTURE DE CAN ===
      // Reconstruction du uint32_t à partir des 4 octets
      uint32_t payload = 
        ((uint32_t)CAN_RX_msg.buf[3] << 24) | 
        ((uint32_t)CAN_RX_msg.buf[2] << 16) | 
        ((uint32_t)CAN_RX_msg.buf[1] << 8) | 
        (uint32_t)CAN_RX_msg.buf[0];
      
      float float_value = payload / 1000.0;  // Conversion en float
      static float pulseTime = 0;
      static float TriggerPulseTime = 10;
      // Affichage pour le debug
      Serial.print("Reçu: payload=");
      Serial.print(payload);
      Serial.print(" => valeur=");
      Serial.println(float_value, 3);  // Affichage avec 3 décimales
      
      // Traitement selon l'ID
      if (CAN_RX_msg.id == 0x1A4 && CAN_RX_msg.len >= 4) {    // 0x1A4 - pwm intake 
        Serial.print("INTAKE: ");
        set_intake_valve_duty(float_value);
        pressureConsign = 0;
        set_pid_enabled(false); 
        send_can(float_value, 0x1A4);
      }
      else if (CAN_RX_msg.id == 0x1A0 && CAN_RX_msg.len >= 4) {    // 0x1A4 - pwm intake 
        Serial.print("PRESSURE CONSIGN: ");
        pressureConsign = float_value;
        set_pid_consign(pressureConsign);                // Consigne 4.55 bar comme votre exemple
        if (pressureConsign) {
          set_pid_enabled(true);
        }
        else {
          set_pid_enabled(false);
          HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_RESET); //set fermé, reset ouvert    
          send_can(1, 0x1A3);
        }
      }
      else if (CAN_RX_msg.id == 0x1A1 && CAN_RX_msg.len >= 4) {    // 0x1A4 - pwm intake 
        Serial.print("FIRE PULSE TIME: ");
        pulseTime = float_value;
      }
      else if (CAN_RX_msg.id == 0x1AF && CAN_RX_msg.len >= 4) {   
        Serial.print("TRIGGER PULSE TIME: ");
        TriggerPulseTime = float_value;
      }
      else if (CAN_RX_msg.id == 0x1A6 && CAN_RX_msg.len >= 4) {    // 0x1A3 - bool exhaust 0-1  
        Serial.print("INTAKE: ");
        if(float_value < 0.5) {
          Serial.print("OFF ");
          set_intake_valve_duty(0);
          send_can(0, 0x1A4);
          set_pid_enabled(false);
        }
        else {
          Serial.print("ON ");
          set_intake_valve_duty(100);
          send_can(99, 0x1A4);
          set_pid_enabled(false);
        }
        pressureConsign = 0;   
      }
      else if (CAN_RX_msg.id == 0x1A3 && CAN_RX_msg.len >= 4) {    // 0x1A3 - bool exhaust 0-1  
        Serial.print("EXHAUST: ");
        if(float_value < 0.5) {
          
          HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_RESET);
          send_can(1, 0x1A3);
          set_pid_enabled(false);
        }
        else {
          
          HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_SET);
          send_can(0, 0x1A3);
          set_pid_enabled(false);
        }
        pressureConsign = 0;
        send_can(float_value, 0x1A3);    
      }
      else if (CAN_RX_msg.id == 0x1A2 && CAN_RX_msg.len >= 4) {    // 0x1A2 - bool expulse 0-1  
        Serial.print("FIRING: ");
        if(float_value < 0.5) {
          Serial.print("OFF ");
          HAL_GPIO_WritePin(FIRING_VALVE_PORT, FIRING_VALVE_PIN, GPIO_PIN_RESET);
        }
        else {
          Serial.print("ON ");
          if (pulseTime == 0){
            HAL_GPIO_WritePin(FIRING_VALVE_PORT, FIRING_VALVE_PIN, GPIO_PIN_SET);
          }
          else {
            if (auto_exhaust) set_pid_enabled(false); //si échapement automatique
            executeFiringSequence(pulseTime, TriggerPulseTime); // trigger de 10ms et un pulse de 50ms
          }
          
        }   
      }
      else if (CAN_RX_msg.id == 0x1A7 && CAN_RX_msg.len >= 4) {    // 0x1A7 - bool trigger 0-1  
        Serial.print("TRIGGER: ");
        if(float_value < 0.5) {
          Serial.print("OFF ");
          HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
        }
        else {
          Serial.print("ON ");
          HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
        } 
          
      }
      else if(CAN_RX_msg.id == 0x1AB && CAN_RX_msg.len >= 4){
         Serial.print("kP: ");
         P = float_value;
         set_pid_parameters(P, I, D, 30.0);  // Kp, Ki, Kd, offset
      }
      else if(CAN_RX_msg.id == 0x1AC && CAN_RX_msg.len >= 4){
         Serial.print("kI: ");
         I = float_value;
         set_pid_parameters(P, I, D, 30.0);  // Kp, Ki, Kd, offset
      }
      else if(CAN_RX_msg.id == 0x1AD && CAN_RX_msg.len >= 4){
         Serial.print("kD: ");
         D = float_value;
         set_pid_parameters(P, I, D, 30.0);  // Kp, Ki, Kd, offset
      }
      else if(CAN_RX_msg.id == 0x1B0 && CAN_RX_msg.len >= 4){
         Serial.print("Auto Exhaust: ");
        if(float_value < 0.5) {
          auto_exhaust = 0;
        }
        else {
          auto_exhaust = 1;
        }
      }
      else if (CAN_RX_msg.id == 0x1B1 && CAN_RX_msg.len >= 4) {   
        Serial.print("delta P: ");
        delta_pressure_compensation = float_value;
      }
      else if (CAN_RX_msg.id == 0x1B2 && CAN_RX_msg.len >= 4) {   
        Serial.print("aplha F: ");
        filter_alpha = float_value;
      }

      HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_SET);
      CAN_msg_received = true;
      
      // Affichage plus détaillé avec la valeur float
      sprintf(buf, "%lu (%.3f)", payload, float_value); 
      Serial.println(buf);
    } 
    else {
      Serial.println("Data: REMOTE REQUEST FRAME");
    }
  }
}

void send_can(uint16_t value, uint16_t can_id) {
  CAN_TX_msg.id = can_id;
  CAN_TX_msg.len = 2;
  CAN_TX_msg.buf[0] = value & 0xFF;         
  CAN_TX_msg.buf[1] = (value >> 8) & 0xFF;  
  Can.write(CAN_TX_msg);
}

void setup() {
  delay(300);
  HAL_Init();
  delay(100);

  Serial.begin(115200);
  while (!Serial){
    HAL_Delay(1);
  } // Attendre que le port soit prêt, sinon le code ne boot pas de façon aléatoire.
  Serial.println("Serial ready");
  

  Can.begin();
  Can.setBaudRate(500000);  //500KBPS
  Serial.println("CAN done");

  // Activer les horloges des GPIOs utilisés
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  setup_pwm();
  set_intake_pwr_valve_duty(0);
  set_intake_ttl_valve_duty(0);

  //PID
  setup_periodic_control(1000);  // Fréquence en Hz
  // Configuration du PID - valeurs de test
  set_pid_parameters(60.0, 40.0, 0.05, 30.0);  // Kp, Ki, Kd, offset
  set_pid_consign(4.55);                // Consigne 4.55 bar comme votre exemple
  set_pid_enabled(false);                // Activer le PID

  // === LEDs ===
  GPIO_InitStruct.Pin = RED_LED_PIN | YELLOW_LED_PIN | GREEN_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // === BUZZER ===
  GPIO_InitStruct.Pin = BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  // === ELECTROMAGNET ===
  /*
  GPIO_InitStruct.Pin = ELECTROMAGNET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ELECTROMAGNET_PORT, &GPIO_InitStruct);
  */
  // -- to do
  GPIO_InitStruct.Pin = FIRING_VALVE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FIRING_VALVE_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(FIRING_VALVE_PORT, FIRING_VALVE_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = EXHAUST_VALVE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EXHAUST_VALVE_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = TRIGGER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIGGER_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET); // mise à 0

  // === Boutons en INPUT ===
  analogReadResolution(12); //max 12. PB0 en conflit avec pa7 BUTTON_EMERGENCY_STOP, à initialiser avant, comme ici
  GPIO_InitStruct.Pin = BUTTON_EMERGENCY_STOP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EMERGENCY_STOP_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = INT_EMERGENCY_STOP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_EMERGENCY_STOP_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = END_STOP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(END_STOP_PORT, &GPIO_InitStruct);
  // -- to do
  GPIO_InitStruct.Pin = EXHAUST_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXHAUST_BUTTON_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);


  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  
  
}

void loop() {
  // Lire les entrées et afficher dans le moniteur série (si activé)
  bool EMERGENCY_STOP = HAL_GPIO_ReadPin(BUTTON_EMERGENCY_STOP_PORT, BUTTON_EMERGENCY_STOP_PIN);
  bool INT_EMERGENCY_STOP = HAL_GPIO_ReadPin(INT_EMERGENCY_STOP_PORT, INT_EMERGENCY_STOP_PIN);
  bool END_STOP = HAL_GPIO_ReadPin(END_STOP_PORT, END_STOP_PIN);
  // -- to do
  bool EXHAUST_BUTTON = HAL_GPIO_ReadPin(EXHAUST_BUTTON_PORT, EXHAUST_BUTTON_PIN);
  //uint16_t PRESSURE_SENSOR = analogRead(PRESSURE_SENSOR_PIN); 
  static bool last_EXHAUST_BUTTON = 0;
  static bool exhaustValue = 0;
  static bool last_INT_EMERGENCY_STOP = 0;

  if (EXHAUST_BUTTON == 0 && last_EXHAUST_BUTTON == 1) {
    exhaustValue = !exhaustValue;
    HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, exhaustValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
    CAN_TX_msg.id = (0x1A3);
    CAN_TX_msg.len = 2; // max8
    CAN_TX_msg.buf[0] = exhaustValue & 0xFF;         // Octet faible
    CAN_TX_msg.buf[1] = (exhaustValue >> 8) & 0xFF;  // Octet fort
    Can.write(CAN_TX_msg);
  }
  last_EXHAUST_BUTTON = EXHAUST_BUTTON;
  
  if (INT_EMERGENCY_STOP == 0 && last_INT_EMERGENCY_STOP == 1) {
    exhaustValue = 0;
    HAL_GPIO_WritePin(EXHAUST_VALVE_PORT, EXHAUST_VALVE_PIN, GPIO_PIN_RESET);
    CAN_TX_msg.id = (0x1A3);
    CAN_TX_msg.len = 2; // max8
    CAN_TX_msg.buf[0] = exhaustValue & 0xFF;         // Octet faible
    CAN_TX_msg.buf[1] = (exhaustValue >> 8) & 0xFF;  // Octet fort
    Can.write(CAN_TX_msg);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  }
  last_INT_EMERGENCY_STOP = INT_EMERGENCY_STOP;

  

  myClock = (millis()/(myClockPeriodMs/2))%2;
  if (myClock == 0 && myPreviousClock == 1){
  
    if (CAN_msg_received = true){
      HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_RESET);
      CAN_msg_received = false;
    }
  }
  myPreviousClock = myClock;
  CAN_Msg_Checking();
}