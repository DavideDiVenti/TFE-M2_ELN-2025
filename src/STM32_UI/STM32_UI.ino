#include "LightLiquidCrystal.h"  // Bibliothèque pour LCD HD44780
#include "STM32_CAN.h"
#include "stm32f3xx_hal.h"
#include <SPIFlash.h>

// LCD PIN
#define LCD_RS PA8
#define LCD_EN PA10
#define LCD_D4 PB1
#define LCD_D5 PB6
#define LCD_D6 PB7
#define LCD_D7 PB0

// Flash
#define CS_PIN   PA0  // Chip Select
#define SCK_PIN  PB3
#define MISO_PIN PB4
#define MOSI_PIN PB5
#define ADDRESS 0x000100  // Adresse mémoire où on stocke le texte

#define BUZZER_PORT GPIOA
#define BUZZER_PIN GPIO_PIN_9

#define RED_LED_PORT GPIOA
#define RED_LED_PIN GPIO_PIN_4

#define YELLOW_LED_PORT GPIOA
#define YELLOW_LED_PIN GPIO_PIN_3

#define GREEN_LED_PORT GPIOA
#define GREEN_LED_PIN GPIO_PIN_1

#define BUTTON_ADC_PIN PA7

// lcd
const uint32_t timeout = 30000; // 20 secondes max
const uint8_t fpsClockPeriodMs = 250;
const uint16_t initClockPeriodMs = 5000;
uint32_t start = 0;



struct CanVariableFloat {
  float value;
  const uint16_t can_id;
  const char* formatStr;
};

struct CanVariableBool {
  bool value;
  const uint16_t can_id;
  const char* formatStr;
};

struct SystemData { 
  // HOME
  CanVariableFloat pressureSensor     = {0.0, 0x1A5, "%d.%02dbar"};

  // MAIN
  CanVariableFloat pressureConsign    = {0.0f, 0x1A0, "%d.%d%dbar"};
  CanVariableFloat PulseTime          = {50,   0x1A1, "%d%d%dms"};

  // MANUAL
  CanVariableFloat intakePWMValue = {0,    0x1A4, "%d%d%%"};
  CanVariableBool intakeValue     = {1,    0x1A6, "OFF ON"};
  CanVariableBool exhaustValue    = {1,    0x1A3, "OFF ON"};
  CanVariableBool firingValue     = {0,    0x1A2, "OFF ON"};
  CanVariableBool triggerValue    = {0,    0x1A7, "OFF ON"};

  // SETTINGS
  CanVariableBool intakeValveType     = {0, 0x1A8, "PWMBIN"};
  CanVariableFloat intakePWMFrequency = {0.5, 0x1A9, "%d.%dkHz"};
  CanVariableBool intakePWMType       = {0, 0x1AA, "TTLPWR"};
  CanVariableFloat P                  = {60, 0x1AB, "%d%d%d.%d%d%d"};
  CanVariableFloat I                  = {40, 0x1AC, "%d%d%d.%d%d%d"};
  CanVariableFloat D                  = {0.05, 0x1AD, "%d%d%d.%d%d%d"};
  CanVariableBool ExhaustValveType    = {0, 0x1AE, " NO NC"};
  CanVariableFloat TriggerPulseTime   = {10, 0x1AF, "%d%dms"};
  CanVariableBool autoExhaust         = {1, 0x1B0, "OFF ON"};
  CanVariableFloat deltaP             = {4.00f, 0x1B1, "%d%d.%d"};
  CanVariableFloat alphaF             = {0.01, 0x1B2, "%d.%d%d"};
};

SystemData sysData;

struct MenuItem {
  const char* title;
  char subtitle[12][13]; // 9 sous-titres, tous à 12 char
};

MenuItem menus[] = {
  {"0-HOME ",    {"1-MAIN      ", "2-MANUAL    ", "3-HISTORY   ", "4-SETTINGS  "}},
  {"1-MAIN ",    {"<-HOME      ", "-Pre 0.00bar", "-Pulse 050ms", "-Firing  OFF"}},
  {"2-MANU.",    {"<-HOME      ", "-Intake  00%", "-Intake  OFF", "-Exhaust  ON", "-Firing  OFF", "-Trigger OFF", "            ", "            "}},
  {"3-HIST.",    {"<-HOME      ", "            ", "            ", "            "}},
  {"4-SETT.",    {"<-HOME      ", "-Intake  PWM", "-PWM  0.4kHz", "-PWM     PWR", "-kP  060.000", "-kI  040.000", "-kD  000.050", "-Exhaust  NO", "-Trigg. 10ms", "-AutoEx.  ON", "-deltaP 04.0", "-aplhaF 0.01"}} //PWM/BIN (si BIN, pas de pid ni pwm), TTL/PWR, NO/NC, 
};

// Pointeurs vers les structures CanVariableFloat uniquement
CanVariableFloat* dataSelectedOnCurrentMenu[5][12] = {  
    {                                  // Menu 0: HOME - pas de variables associées
        NULL, NULL, NULL, NULL, NULL
    },
    {                                  // Menu 1: MAIN
        NULL,                          // <-HOME
        &sysData.pressureConsign,      // -Pre ?.??bar
        &sysData.PulseTime,            // -Pulse ???ms
        NULL                           // -Firing ??? (bool, donc NULL)
    },
    {                                  // Menu 2: MANUAL
        NULL,                          // <-HOME
        &sysData.intakePWMValue,       // -Intake ??%
        NULL,                          // -Intake ??? (bool)
        NULL,                          // -Exhaust ??? (bool)
        NULL,                          // -Firing ??? (bool)
        NULL                           // -Trigger ??? (bool)
    },
    {                                  // Menu 3: HISTORY
        NULL, NULL, NULL, NULL
    },
    {                                   // Menu 4: SETTINGS
        NULL,                           // <-HOME
        NULL,                           // -Firing ??? (bool)
        &sysData.intakePWMFrequency,    // -PWM ???kHz
        NULL,                           // -PWM ??? (bool)
        &sysData.P,                     // -P ????.???
        &sysData.I,                     // -I ????.???
        &sysData.D,                     // -D ????.???
        NULL,                           // -Exhaust ??? (bool)
        &sysData.TriggerPulseTime,      // -Trigg. ??ms
        NULL,                           // -autoexhaust
        &sysData.deltaP,                // -
        &sysData.alphaF                 // -

    }
};

// menu1-subtitle3 -> PulseTime (SystemData)
// 0 Nothing, 1 MAIN : 1.0 [/]; 1.1 [pressureConsign]; 1.2 [PulseTime],; 1.3 [firingValue], 2 MANU : 2.0 [/]; 2.1 [intakePWMValue] ; 2.2 [intakeValue], ... , 3 HIST
//dataSelectedOnCurrentMenu[][] = {{/},{/,pressureConsign,PulseTime,firingValue}, {intakePWMValue,intakeValue, ..., ..., ...}, {...}, {...}}

// Buttons
bool buttonEvent = false;  // Variable pour l'interruption
char buttonPressed[5];
uint8_t cursorLine = 0;
uint8_t currentMenu = 0; // 0-HOME; 1-MAIN; 2-MANUAL; 3-HISTORY; 4-SETTINGS
uint8_t subMenu = 0; // no subMenu activated if 0

bool CAN_msg_rxtx = false;


float countdownBeforeFiring = 3;
bool startCountdownBeforeFiring = 0;

// objects
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
STM32_CAN Can(CAN1, DEF);
SPIFlash flash(CS_PIN);
UART_HandleTypeDef huart2;

// Fonction d'impression minimaliste directe
void HAL_UART_Print(const char* msg) {
  if (msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  }
}


void Cursor_LCD_Content(){
  // cursor blinking
  for (uint8_t i=0; i<4; i++){
    lcd.setCursor(7, i);
    lcd.print("|");
  }
  
  lcd.setCursor(7, cursorLine%4);
  if ((HAL_GetTick()/250)%2 == 0) {
    if (subMenu == 0) lcd.print(">");
    if (subMenu != 0) lcd.print("+");
  } else{
    if (subMenu == 0) lcd.print("|");
    if (subMenu != 0) lcd.print("-");
  }

  // Digit blinking
  static uint8_t prevSubMenu = 0;
  lcd.setCursor(8, cursorLine%4);
  if (subMenu != 0) {
    if ((HAL_GetTick()/250)%2 == 0) {
      lcd.print(menus[currentMenu].subtitle[cursorLine]); //submenu tout entier
    }
    else{
      char tempStr[21];
      strcpy(tempStr, menus[currentMenu].subtitle[cursorLine]);
      int digitCount = 0;
      for (int i = 0; tempStr[i] != '\0'; i++) {
        if (isdigit(tempStr[i])) {
          digitCount++;
          if (digitCount == subMenu) {
            tempStr[i] = ' ';
            break;
          }
        }
      }
      lcd.print(tempStr);
    }
  }
  
  else if (prevSubMenu != 0){ //pour éviter qu'à la sortie, ça reste avec un digit blanc
    lcd.print(menus[currentMenu].subtitle[cursorLine]);
  }
  prevSubMenu = subMenu;
}


void Button_LCD_Content(){
  if (buttonEvent == true){
    lcd.setCursor(0, 3);
    lcd.print(buttonPressed);
    //HAL_UART_Print(buttonPressed);
    //HAL_UART_Print("\r\n");
    Menu_Managment();    
    
    buttonEvent = false;
  }
}

const char* dataFormatting(CanVariableFloat* var){
  // Appelé pour formater une data int en char avec unité -> print sur le lcd
  // Mais aussi, appelé pour extraire plus facilement les digit depuis ce char -> edit on lcd
  const char *str = var->formatStr;
  static char buf[32];
  // 1) Comptage de digits totaux
  uint8_t digits = 0;
  for (int i = 0; str[i] != '\0'; i++) {
    if (str[i] == 'd') {
      digits++;
    }
  }

  // 2) position du point
  uint8_t pointPos = 0;
  for (int i = 0; str[i] != '\0'; i++) {
    if (str[i] == '.') {
      pointPos = (int)i/2;
      break; // On sort dès qu'on trouve le point
    }
  }
  if (pointPos == 0) pointPos = digits;
  
  // 3) construction brute
  uint8_t leftDigits = pointPos;
  uint8_t rightDigits = digits - pointPos;
  uint32_t left = (int)var->value;
  uint32_t right = (uint32_t)( ((var->value - left) * pow(10, rightDigits)) + 0.5f );

  // 4) ajout de l'unité
  const char *unit = str;
  int formatLen = strlen(str);
  int expectedFormatLength = digits * 2; // Avancer jusqu'à la fin des formats %d (2 caractères chacun) + 1 pour le point si présent
  if (strchr(str, '.')) expectedFormatLength += 1;

  if (formatLen > expectedFormatLength) unit = &str[expectedFormatLength];  // Il y a quelque chose après les formats : unité détectée   
  else unit = ""; // Pas d'unité

  // print
  if (pointPos == digits) sprintf(buf, "toStr: %0*d%s\r\n", leftDigits, left, unit);
  else sprintf(buf, "toStr: %0*d.%0*d%s\r\n", leftDigits, left, rightDigits, right, unit);
  // Corriger '%%' à la fin de la chaîne en '%'
  //HAL_UART_Print(buf);
  return buf;
}

// Fonction pour obtenir le pointeur vers la structure CanVariableFloat sélectionnée
CanVariableFloat* getSelectedFloatVariable(uint8_t currentMenu, uint8_t cursorLine) {
    return dataSelectedOnCurrentMenu[currentMenu][cursorLine];
}

// il faut faire blinker aussi
void Digit_Managment(int8_t sign){
  CanVariableFloat* selectedVar = getSelectedFloatVariable(currentMenu, cursorLine);
  char textData[9]; // Déclarez d'abord sans initialisation
  strcpy(textData, dataFormatting(selectedVar)); // 0.64 -> "0.64kHz" ; 0153.215 -> "0153.215" ; ...
  //HAL_UART_Print(textData); // I intakePWMValue firingPWMFrequency 

  
  uint8_t digits[7]; // Tableau pour stocker les chiffres extraits en char
  uint8_t digitCount = 0;
  int8_t decimalPosition = -1; // -1 signifie qu'aucune virgule n'a été trouvée

  for (uint8_t i = 0; i < strlen(textData) && digitCount < 7; i++) {
      if (textData[i] >= '0' && textData[i] <= '9') {        
          digits[digitCount] = textData[i] - '0'; // Convertir le caractère en chiffre
          digitCount++;
      }
      else if ((textData[i] == '.')) {
          decimalPosition = digitCount; // Enregistrer la position de la virgule
      }
  }

  // Indexation à zéro pour les chiffres (subMenu commence à 1)
  uint8_t digitIndex = subMenu - 1;
  
  // Modifier le chiffre sélectionné
  /*
  if ((sign > 0 && digits[digitIndex] < 9) || (sign < 0 && digits[digitIndex] > 0)) {
    digits[digitIndex] += sign;
  }
  */
  if ((sign > 0 && digits[digitIndex] == 9)){
    digits[digitIndex] = 0;
  }
  else if (sign < 0 && digits[digitIndex] == 0){
    digits[digitIndex] = 9;
  }
  else {
    digits[digitIndex] += sign;
  }

  // Construction de la chaîne formatée dynamiquement en fonction du nombre de chiffres
  char numberUpdatedText[50]; // Taille suffisante pour tous les formats
  
  // Appliquer le format approprié en fonction du nombre de chiffres
  switch (digitCount) {
    case 1: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0]); break;
    case 2: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1]); break;
    case 3: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2]); break;
    case 4: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2], digits[3]); break;
    case 5: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2], digits[3], digits[4]); break;
    case 6: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2], digits[3], digits[4], digits[5]); break;
    case 7: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2], digits[3], digits[4], digits[5], digits[6]); break;
    case 8: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2], digits[3], digits[4], digits[5], digits[6], digits[7]); break;
    case 9: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2], digits[3], digits[4], digits[5], digits[6], digits[7], digits[8]); break;
    case 10: sprintf(numberUpdatedText, selectedVar->formatStr, digits[0], digits[1], digits[2], digits[3], digits[4], digits[5], digits[6], digits[7], digits[8], digits[9]); break;
  }
  
  selectedVar->value = atof(numberUpdatedText); //atoi ignore tout caractère non numérique apr!s les chiffres
  lcd.setCursor(20-strlen(numberUpdatedText), cursorLine%4);
  lcd.print(numberUpdatedText);
  refreshMenuStruct(numberUpdatedText);
  
  //HAL_UART_Print("\r\n");
  CAN_Msg_Write(selectedVar->can_id, selectedVar->value);
}

void CAN_Msg_Write(uint16_t can_id, float value) {
  static CAN_message_t CAN_TX_msg;
  
  // Limiter la valeur pour éviter les dépassements (environ 4.2 millions)
  if (value < 0) value = 0;
  if (value > 4294967.0) value = 4294967.0;  
  
  // Conversion en uint32_t
  uint32_t payload = (uint32_t)(value * 1000.0 + 0.5);
  
  // Configuration du message CAN
  CAN_TX_msg.id = can_id;
  CAN_TX_msg.len = 4;  // 4 octets au lieu de 2
  
  // Remplissage du buffer octet par octet (little-endian)
  CAN_TX_msg.buf[0] = (uint8_t)(payload & 0xFF);
  CAN_TX_msg.buf[1] = (uint8_t)((payload >> 8) & 0xFF);
  CAN_TX_msg.buf[2] = (uint8_t)((payload >> 16) & 0xFF);
  CAN_TX_msg.buf[3] = (uint8_t)((payload >> 24) & 0xFF);
  
  // Envoi du message
  Can.write(CAN_TX_msg);
  
  // Signalisation
  HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_SET);
  CAN_msg_rxtx = true;
}

void Refresh_Menu(uint8_t index = currentMenu) {
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(menus[index].title);
  currentMenu = index;
  
  uint8_t totalItems = getSubmenuCount(); // total de sous-titres
  uint8_t pageStart = (cursorLine / 4) * 4;    // début de la page (0, 4, 8, etc.)

  for (int i = 0; i < 4; i++) {
    int itemIndex = pageStart + i;
    if (itemIndex < totalItems) {
      lcd.setCursor(8, i);
      lcd.print(menus[index].subtitle[itemIndex]);
    }
  }
}

void Init_LCD(){
  static bool initClock = 0; 
  static bool previousInitClock = 0;
  
  if ((HAL_GetTick() - start) < timeout){
    initClock = (HAL_GetTick()/(initClockPeriodMs/2))%2;
    if (initClock == 1 && previousInitClock == 0) {
      lcd.begin(20, 4);
      Refresh_Menu();
    }
    previousInitClock = initClock;
  }
}

uint8_t getSubmenuCount() {
  uint8_t count = 0;
  for (int i = 0; i < 12; i++) { // ← augmente si tu peux avoir plus de 4 sous-titres
    if (strlen(menus[currentMenu].subtitle[i]) > 0) count++;
  }
  return count;
}

void Menu_Managment(){ 
  switch(buttonPressed[0]){
    case 'U' :
      if (cursorLine > 0 && subMenu == 0) cursorLine--; // on voyage de haut en bas les menus/sous menus
      if (cursorLine == 3 && subMenu == 0) Refresh_Menu(); // page 0
      if (cursorLine == 7 && subMenu == 0) Refresh_Menu(); // page 1  
      if (cursorLine == 11 && subMenu == 0) Refresh_Menu(); // page 2   
      if (subMenu!=0) Digit_Managment(1); // on change les chiffres des sous menu
      break;
    case 'D' : 
      if (cursorLine < (getSubmenuCount()-1) && subMenu == 0) cursorLine++; 
      if (cursorLine == 4 && subMenu == 0) Refresh_Menu(); // page 1
      if (cursorLine == 8 && subMenu == 0) Refresh_Menu(); // page 2
      if (cursorLine == 12 && subMenu == 0) Refresh_Menu(); // page 3
      if (subMenu!=0) Digit_Managment(-1);
      break;
    case 'O' : 
      if (currentMenu == 0) {Refresh_Menu(cursorLine+1); cursorLine =0; }// nouveau menu
      else if (currentMenu != 0 && cursorLine == 0) Refresh_Menu(0); // retour au HOME
      else if (currentMenu != 0 && cursorLine > 0) Submenu_Managment(currentMenu); // OK sur un subMenu
      break;
    case ' ' :
      if ((currentMenu == 1) && (cursorLine == 3)){ // menu MAIN - firing
        countdownBeforeFiring = 3;
        startCountdownBeforeFiring = 0;
        lcd.setCursor(17, 3);
        lcd.print("OFF");
      }   
  }
  char buf[10];
  sprintf(buf, "%d ", cursorLine); 
  //HAL_UART_Print(buf);
}

void firingProcedure(){

  char statusStr[4];

  sysData.firingValue.value = !sysData.firingValue.value;
  strncpy(statusStr, sysData.firingValue.formatStr + (sysData.firingValue.value ? 3 : 0), 3);
  statusStr[3] = '\0'; 
  lcd.setCursor(17, 3);
  lcd.print(statusStr); // affichage du ON avant cette pause

  CAN_Msg_Write(sysData.PulseTime.can_id, sysData.PulseTime.value);
  CAN_Msg_Write(sysData.firingValue.can_id, sysData.firingValue.value);

  HAL_Delay(sysData.PulseTime.value); // pause

  sysData.firingValue.value = !sysData.firingValue.value; // rafraichissement de ON vers OFF
  strncpy(statusStr, sysData.firingValue.formatStr + (sysData.firingValue.value ? 3 : 0), 3); 
  statusStr[3] = '\0'; // pour s'assurer que c'est bien terminé
  lcd.setCursor(17, 3);
  lcd.print(statusStr); // affichage du OFF
  refreshMenuStruct(statusStr);
  
}

void countDownLoop(){
  if (startCountdownBeforeFiring == 1){    
    char statusStr[5];
    int sec = (int)countdownBeforeFiring;
    int cent = (int)((countdownBeforeFiring - sec) * 10);
    sprintf(statusStr, "%d.%d", sec, cent);
    lcd.setCursor(17, 3);
    lcd.print(statusStr);
    if ((int)(countdownBeforeFiring*1000) % 1000 == 0){
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
      HAL_Delay(30);
      if (countdownBeforeFiring ==0) HAL_Delay(170);
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    }   
    if (countdownBeforeFiring == 0){
      startCountdownBeforeFiring = 0;
      firingProcedure(); 
      // actualiser la vraie pression avant tir   
    }    
    countdownBeforeFiring -= (float)fpsClockPeriodMs/1000;   
  }
}

void Submenu_Managment(uint8_t menu){
  // clické sur un submenu
  char statusStr[4]; // 3 lettres + null terminator
  switch (menu){
    case 1 : // MAIN
      switch(cursorLine){
          case 1 : // Pressure
            subMenu++;
            if (subMenu >3) subMenu = 0; //max 3 digit
          break;
          case 2 : // impulse
            subMenu++;
            if (subMenu >3) subMenu = 0; //max 3 digit
          break;
          case 3 : // firing
            countdownBeforeFiring = 3;
            startCountdownBeforeFiring = 1;   
          break;      
      }

      break;
    case 2 : // MANUAL
      switch(cursorLine){
        
        case 1 : // Intake
          subMenu++;
          if (subMenu >2) subMenu = 0; //max 2 digit
          sysData.intakeValue.value = 0;
          strncpy(statusStr, sysData.intakeValue.formatStr + (sysData.intakeValue.value ? 3 : 0), 3);
          statusStr[3] = '\0'; // pour s'assurer que c'est bien terminé
          lcd.setCursor(17, 2);
          lcd.print(statusStr);
          
          break;
        case 2 : // Intake bool
          sysData.intakeValue.value = !sysData.intakeValue.value;  // toggle 0 ⇄ 1
          CAN_Msg_Write(sysData.intakeValue.can_id, sysData.intakeValue.value);         
          strncpy(statusStr, sysData.intakeValue.formatStr + (sysData.intakeValue.value ? 3 : 0), 3);
          statusStr[3] = '\0'; // pour s'assurer que c'est bien terminé
          lcd.setCursor(17, 2);
          lcd.print(statusStr);
          refreshMenuStruct(statusStr);
          sysData.intakePWMValue.value = 99*sysData.intakeValue.value;
          sprintf(statusStr, "%02d%%", (int)sysData.intakePWMValue.value);
          statusStr[4] = '\0'; // pour s'assurer que c'est bien terminé
          lcd.setCursor(17, 1);
          lcd.print(statusStr);
          
          break;
        case 3 : // Exhaust
          sysData.exhaustValue.value = !sysData.exhaustValue.value;
          CAN_Msg_Write(sysData.exhaustValue.can_id, sysData.exhaustValue.value);          
          strncpy(statusStr, sysData.exhaustValue.formatStr + (sysData.exhaustValue.value ? 3 : 0), 3);
          statusStr[3] = '\0'; // pour s'assurer que c'est bien terminé
          lcd.setCursor(17, 3);
          lcd.print(statusStr);
          refreshMenuStruct(statusStr);
          break;
        case 4 : // firing
          sysData.firingValue.value = !sysData.firingValue.value; 
          CAN_Msg_Write(sysData.PulseTime.can_id, 0);
          CAN_Msg_Write(sysData.firingValue.can_id, sysData.firingValue.value);
          strncpy(statusStr, sysData.firingValue.formatStr + (sysData.firingValue.value ? 3 : 0), 3);
          statusStr[3] = '\0'; // pour s'assurer que c'est bien terminé
          lcd.setCursor(17, 0);
          lcd.print(statusStr);
          refreshMenuStruct(statusStr);
          break;
        case 5 : // TRIGGER
          sysData.triggerValue.value = !sysData.triggerValue.value; 
          CAN_Msg_Write(sysData.TriggerPulseTime.can_id, 0);
          CAN_Msg_Write(sysData.triggerValue.can_id, sysData.triggerValue.value);
          strncpy(statusStr, sysData.triggerValue.formatStr + (sysData.triggerValue.value ? 3 : 0), 3);
          statusStr[3] = '\0'; // pour s'assurer que c'est bien terminé
          lcd.setCursor(17, 1);
          lcd.print(statusStr);
          refreshMenuStruct(statusStr);
          break;
      } 
      break;
    case 3 : // HISTORY
      
      break;
    case 4 : // SETTINGS
      switch(cursorLine){
        case 4 : // P
          subMenu++;
            if (subMenu >6) subMenu = 0; //max 2 digit  
        break;
        case 5 : // I
          subMenu++;
            if (subMenu >6) subMenu = 0; //max 2 digit  
        break;
        case 6 : // D
          subMenu++;
            if (subMenu >6) subMenu = 0; //max 2 digit  
        break;
        case 8 : // TRIGGER TIME
          subMenu++;
            if (subMenu >2) subMenu = 0; //max 2 digit  
        break;
        case 9 : // AUTO EXHAUST
          sysData.autoExhaust.value = !sysData.autoExhaust.value;
          CAN_Msg_Write(sysData.autoExhaust.can_id, sysData.autoExhaust.value);          
          strncpy(statusStr, sysData.autoExhaust.formatStr + (sysData.autoExhaust.value ? 3 : 0), 3);
          statusStr[3] = '\0'; // pour s'assurer que c'est bien terminé
          lcd.setCursor(17, 1);
          lcd.print(statusStr);
          refreshMenuStruct(statusStr);
        break;
        case 10 : // DELTA P
          subMenu++;
            if (subMenu >3) subMenu = 0; //max 3 digit  
        break;
        case 11 : // ALPHA F
          subMenu++;
            if (subMenu >3) subMenu = 0; //max 3 digit  
        break;
      }
      break;
  }
  

}

void refreshMenuStruct(char* statusStr){
  int valueLen = strlen(statusStr);
  int startPos = 12 - valueLen;
  if (startPos < 0) startPos = 0; // Sécurité
  strcpy(&menus[currentMenu].subtitle[cursorLine][startPos], statusStr);
}
  
void handleButtonPress() {
  static uint8_t adcValue = 0;

  detachInterrupt(digitalPinToInterrupt(BUTTON_ADC_PIN)); // désactivation temporaire
  adcValue = analogRead(BUTTON_ADC_PIN);
  pinMode(BUTTON_ADC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_ADC_PIN), handleButtonPress, CHANGE);
  
  uint8_t newButton = (7 - (adcValue / 10)); // 3-UP ; 2-DOWN ; 1-OK ; 0-void
    buttonEvent = true;
    switch (newButton) {
      case 1:  strcpy(buttonPressed, "OK  "); break;
      case 2:  strcpy(buttonPressed, "DOWN"); break;
      case 3:  strcpy(buttonPressed, "UP  "); break;
      default: strcpy(buttonPressed, "    "); break;
    }    
}

void CAN_Msg_Read() {
  static CAN_message_t CAN_RX_msg;
  if (Can.read(CAN_RX_msg)) {
    char buf[64];
    uint16_t data = 0;
    sprintf(buf, "Channel:%d ", CAN_RX_msg.bus); // interface CAN1/CAN2 chez le stm32
    //HAL_UART_Print(buf);
    
    if (CAN_RX_msg.flags.extended == false) { 
      //HAL_UART_Print("Standard ID:"); // ID sous 11 bit (ex: 0x7FF) max 2048 ID
    } else {
      //HAL_UART_Print("Extended ID:"); // ID sous 29 bit (ex : 0x18DAF100) max 536 870 912 ID
    }
    
    sprintf(buf, "%X DLC: %d ", CAN_RX_msg.id, CAN_RX_msg.len); // max 8 octets (data lenght code)
    //HAL_UART_Print(buf);
    
    if (CAN_RX_msg.flags.remote == false) {
      //HAL_UART_Print("buf: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        sprintf(buf, "0x%X ", CAN_RX_msg.buf[i]);
        //HAL_UART_Print(buf);
      }
      //HAL_UART_Print("\r\n");
      // === LECTURE DE LA VALEUR ADC SI ID = 0x1A5 ===
      static float pressure = 0;
      static uint16_t pwm = 0;
      static bool exhaust = 0;
      if (CAN_RX_msg.id == sysData.pressureSensor.can_id && CAN_RX_msg.len >= 2) {
        data = (CAN_RX_msg.buf[1] << 8) | CAN_RX_msg.buf[0];
        float pressure_bar = (data / 4095.0f) * 9.99f;
        int pression_entier = pressure_bar; // partie entière
        int pression_decimal = (pressure_bar - pression_entier) * 100; // deux chiffres après la virgule
        sprintf(buf, sysData.pressureSensor.formatStr, pression_entier, pression_decimal);
        lcd.setCursor(0, 1);
        lcd.print(buf);
        if (pressure_bar <= 0.5){
          HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
        }
        else {
          HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
        }
        pressure = pressure_bar;
        // pressure
        HAL_UART_Print(buf);
        HAL_UART_Print(";");
        // consigne
        // Récupérer la chaîne du menu
        char* menuString = menus[1].subtitle[1]; // "-Pre 0.00bar"
        char* valueStart = strstr(menuString, " ") + 1; // Trouve le premier espace et avance de 1
        HAL_UART_Print(valueStart); // Affiche "0.00bar"
        HAL_UART_Print(";");
        // pwm
        sprintf(buf, "%02u%%", pwm);
        HAL_UART_Print(buf);
        HAL_UART_Print(";");
        // exhaust
        sprintf(buf, "%01u", exhaust);
        HAL_UART_Print(buf);
        HAL_UART_Print("\n");
        
      }
      if (CAN_RX_msg.id == sysData.exhaustValue.can_id && CAN_RX_msg.len >= 2) {
        data = (CAN_RX_msg.buf[1] << 8) | CAN_RX_msg.buf[0];
        const char* baseStr = sysData.exhaustValue.formatStr; // " ONOFF"  
        strncpy(buf, baseStr + (data ? 3 : 0), 3);
        buf[3] = '\0'; // Null-terminate
        lcd.setCursor(4, 2);
        lcd.print(buf);
        exhaust = data;
      }
      if (CAN_RX_msg.id == sysData.intakePWMValue.can_id && CAN_RX_msg.len >= 2) {
        data = (CAN_RX_msg.buf[1] << 8) | CAN_RX_msg.buf[0];
        if (data>99) data = 99;
        sprintf(buf, "%02u%%", data);  // Conversion en décimal
        lcd.setCursor(0, 2);
        lcd.print(buf);
        pwm = data;
      }
    } 
    else {
      //HAL_UART_Print("Data: REMOTE REQUEST FRAME\r\n");
    }
  }
}

void setup() {
  // Initialisation HAL minimale
  HAL_Init();
  HAL_Delay(10);
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // Initialisation UART minimale
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  
  __HAL_RCC_USART2_CLK_ENABLE();
  HAL_UART_Init(&huart2);
  
  HAL_UART_Print("Serial ready\r\n");
  HAL_Delay(10);
  
  
  Can.begin();
  Can.setBaudRate(500000);
  HAL_UART_Print("CAN ready\r\n");
  
  // prévoir un code qui écris les données dans la flash depuis l'entrée UART.
  // ex : flash set adress;payload,adress;payload,adress;payload,... flash delete all, flash delete adress4Ko, ...
  // et inversemet, au meme format.recupérer ce qui est dans la flash 
  // ex : flash get all, flash get adress, ...
  // comme un gcode, une suite d'instruction
  SPI.begin();
  HAL_Delay(10);
  HAL_UART_Print("SPI ready\r\n");
  
  if (flash.initialize()) {
    HAL_UART_Print("- W25Q64 found\r\n");
    
    // Texte à stocker
    const char message[] = "Hello STM32!";
    size_t msgSize = sizeof(message);
    
    flash.blockErase4K(ADDRESS);
    flash.writeBytes(ADDRESS, (uint8_t*)message, msgSize);
    
    char readBuffer[msgSize] = {0};
    flash.readBytes(ADDRESS, (uint8_t*)readBuffer, msgSize);
    
    HAL_UART_Print("- Text read: ");
    HAL_UART_Print(readBuffer);
    HAL_UART_Print("\r\n");
  } 
  else {
    HAL_UART_Print("FLASH error\r\n");
  }


  GPIO_InitStruct.Pin = BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GREEN_LED_PIN;
  HAL_GPIO_Init(GREEN_LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = YELLOW_LED_PIN;
  HAL_GPIO_Init(YELLOW_LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RED_LED_PIN;
  HAL_GPIO_Init(RED_LED_PORT, &GPIO_InitStruct);

  pinMode(BUTTON_ADC_PIN, INPUT);
  HAL_UART_Print("PINOUT ready\r\n");

  // Test des LEDs
  HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
  
  // Configuration de l'ADC
 
  
  
  // Configuration de l'interruption
  attachInterrupt(digitalPinToInterrupt(BUTTON_ADC_PIN), handleButtonPress, CHANGE);
  HAL_UART_Print("- INT enabled\r\n");
  analogReadResolution(6);
  HAL_UART_Print("- ADC 6 bit\r\n");


  start = HAL_GetTick();
}

void loop() {
  static bool fpsClock = 0; // variable locale qui persiste au delà de l'exécution de la fonction (garde sa valeur entre appels)
  static bool previousfpsClock = 0;

  Button_LCD_Content();
  // boucle fps
  fpsClock = (HAL_GetTick()/(fpsClockPeriodMs/2))%2;
  if (fpsClock == 0 && previousfpsClock == 1) {
    Init_LCD(); // 30 première secondes, initialisation du lcd
    Cursor_LCD_Content();  // gestion du LCD tout entier depuis le curseur
    countDownLoop(); // compte à rebours avant tir
    if (CAN_msg_rxtx = true){
      HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_RESET);
      CAN_msg_rxtx = false;
    }
  } 
  previousfpsClock = fpsClock;
  
  // scrutation CAN
  CAN_Msg_Read();
}
