#ifndef PERIODIC_CONTROL_H
#define PERIODIC_CONTROL_H

#include "stm32f3xx_hal.h"
#include <Arduino.h>

// === Déclarations des variables globales ===
extern HardwareTimer *MyTim;
extern volatile int control_counter;

// Variables PID
extern float pid_consign;
extern float pid_kp, pid_ki, pid_kd;
extern bool pid_enabled;

// === Prototypes des fonctions ===

/**
 * @brief Configure et démarre le timer pour la fonction de contrôle périodique
 * @param frequency_hz Fréquence d'exécution en Hz (ex: 1000 pour 1ms)
 */
void setup_periodic_control(uint32_t frequency_hz);

/**
 * @brief Fonction de contrôle exécutée périodiquement par interruption
 * Cette fonction sera appelée automatiquement à la fréquence configurée
 */
void periodic_control_function(void);

/**
 * @brief Configure les paramètres PID avec offset
 * @param kp Gain proportionnel
 * @param ki Gain intégral  
 * @param kd Gain dérivé
 * @param offset Offset d'activation électrovanne (%)
 */
void set_pid_parameters(float kp, float ki, float kd, float offset = 30.0f);

/**
 * @brief Active/désactive le contrôle PID
 * @param enabled true pour activer, false pour désactiver
 */
void set_pid_enabled(bool enabled);

/**
 * @brief Définit la consigne de pression
 * @param consign Consigne en bar (0-10)
 */
void set_pid_consign(float consign);

/**
 * @brief Retourne la valeur actuelle du compteur de contrôle
 * @return Valeur du compteur (pour debug)
 */
int get_control_counter(void);

/**
 * @brief Remet à zéro le compteur de contrôle
 */
void reset_control_counter(void);

#endif // PERIODIC_CONTROL_H