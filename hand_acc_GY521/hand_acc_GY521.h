#ifndef HAND_ACC_GY521_H_
#define HAND_ACC_GY521_H_

#include <stdint.h>

typedef struct{
	float ax;
	float ay;
	float az;
	float temp;
}MPU6050_data_out;

/* [REVIEW-FIX:7] MPU6050_Init ora ritorna lo stato del WHO_AM_I check invece di
 * chiamare Error_Handler() (che bloccava tutto il dispositivo: BLE, MEMS,
 * kinematics). Il main può decidere se continuare senza palmo o ritentare.
 *   0 = OK
 *   1 = WHO_AM_I errato o I2C timeout
 * Per ripristinare il vecchio comportamento (fatal) basta ignorare il return
 * value e gestire l'errore con Error_Handler() lato chiamante. */
uint8_t MPU6050_Init(void);

/* [REVIEW-FIX:7] MPU6050_Read ritorna 0 in caso di successo, 1 se la lettura
 * I2C fallisce. Quando fallisce, data_out NON viene aggiornato e il filtro
 * EMA mantiene l'ultimo valore valido. */
uint8_t MPU6050_Read(MPU6050_data_out* data_out);

/* [REVIEW-FIX:7] Espone lo stato di salute del sensore. Permette al main / a
 * BLE_Stream di trasmettere un palm_rel_deg "non valido" (es. NaN) quando
 * l'MPU non risponde, invece di propagare l'ultimo campione stantio. */
uint8_t MPU6050_IsHealthy(void);


#endif //HAND_ACC_GY521_H_
