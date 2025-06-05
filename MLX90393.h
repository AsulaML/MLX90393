#ifndef MLX90393_H
#define MLX90393_H

#include <stdint.h>

// ---- Registres du MLX90393 (cf. datasheet) ----
#define GAIN_SEL_REG       0x00
#define TCMP_EN_REG        0x01
#define OSR_REG            0x02
#define SENS_TC_LT_REG     0x03
#define X_OFFSET_REG       0x04
#define Y_OFFSET_REG       0x05
#define Z_OFFSET_REG       0x06

// ---- Masques pour configuration ----
#define DIG_FLT_MASK       0x00F0
#define OSR_MASK           0x000F

// ---- Modes de gestion de l’offset ----
#define NO_OFFSET          0
#define APPLY_OFFSET       1
#define SAVE_OFFSET        2

// ---- Prototypes ----

/**
 * Initialise le capteur MLX90393 avec configuration rapide ou lente.
 * @param Tint 0 = rapide (moins précis), 1 = lent (plus précis)
 */
void init_MLX90393(uint8_t Tint);

/**
 * Lit les axes X, Y, Z et la température.
 * @param X Pointeur vers la valeur X retournée
 * @param Y Pointeur vers la valeur Y retournée
 * @param Z Pointeur vers la valeur Z retournée
 * @param T Pointeur vers la valeur de température brute
 * @param EnableOffset Mode de gestion des offsets (NO_OFFSET, APPLY_OFFSET, SAVE_OFFSET)
 */
void GetXYZT(int16_t* X, int16_t* Y, int16_t* Z, uint16_t* T, uint8_t EnableOffset);

/**
 * Vérifie si le capteur est détecté et actif.
 * @return 1 si détecté, 0 sinon
 */
uint8_t IsAlive(void);

/**
 * Lecture d’un registre MLX90393.
 * @param address Adresse du registre
 * @param data Pointeur pour recevoir les données (16 bits)
 * @return Status byte retourné par le capteur
 */
uint8_t ReadRegister(uint8_t address, uint16_t* data);

/**
 * Écriture dans un registre MLX90393.
 * @param address Adresse du registre
 * @param data Donnée à écrire (16 bits)
 * @return Status byte retourné par le capteur
 */
uint8_t WriteRegister(uint8_t address, uint16_t data);

/**
 * Lecture d’un "status" rapide (à partir de GAIN_SEL_REG)
 */
uint8_t ReadStatus(void);

#endif // MLX90393_H
