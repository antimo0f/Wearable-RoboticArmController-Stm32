#include "ble_stream.h"

#include <string.h>
#include <math.h>   /* [REVIEW-FIX:7] per NAN, usato come "palm non disponibile" */

#include "stm32f4xx_hal.h"
#include "app_state.h"
#include "gatt_db.h"
#include "bluenrg1_aci.h"

#include "mems_kinematics.h"
#include "arm_palm.h"
#include "hand_acc_GY521.h"   /* [REVIEW-FIX:7] per MPU6050_IsHealthy() */
#include "flex_sensor.h"      /* striscia ZD10-100 -> stato pinza */



/* Variabili definite in app_bluenrg_2.c */
extern volatile uint16_t connection_handle;

/* Frequenza di trasmissione (ms). 20 ms = 50 Hz. */
#define BLE_STREAM_PERIOD_MS    20U

/* Magic byte per identificare il pacchetto lato client */
#define BLE_STREAM_MAGIC        0xA5U

/* Layout pacchetto (little-endian, packed):
 *   uint8_t  magic         (1)
 *   uint8_t  seq           (1)
 *   uint32_t timestamp     (4)   ms
 *   float    ypr[3]        (12)  yaw, pitch, roll del braccio (deg)
 *   float    palm_rel_deg  (4)   pitch palmo relativo (deg), NaN = MPU palmo KO
 *   uint8_t  close_pinza   (1)   posizione pinza (0..255), 0 = aperta
 * Totale: 23 byte
 */
#define BLE_STREAM_PAYLOAD_LEN  23U

static MPU6050_data_out s_mpu_last;
static uint32_t s_last_tx_tick = 0U;
static uint8_t  s_seq = 0U;

/* [WIRED:B1] get_pinza_position: ora ritorna il flag "touched" della striscia
 * di pressione ZD10-100 (PC2/PC3, modulo flex_sensor). 1 = pinza chiusa,
 * 0 = pinza aperta. Il valore viene aggiornato da FlexSensor_Process() nel
 * main loop, quindi qui basta leggere lo stato corrente.
 * Storia precedente:
 *  1) collegare davvero la lettura della pinza qui (sensore/encoder) → il byte
 *     acquisisce significato;
 *  2) togliere il byte dal payload e ridurre BLE_STREAM_PAYLOAD_LEN da 23 a 22.
 * Lascio la funzione attiva perché il pacchetto la usa: rimuoverla rompe il
 * layout BLE. Marcata come morta per ricordare il TODO. */
uint8_t get_pinza_position(void){
	const FlexSensor_Data *fx = FlexSensor_Get();
	return (fx != NULL) ? fx->touched : 0U;
}

void BLE_Stream_Init(void)
{
    memset(&s_mpu_last, 0, sizeof(s_mpu_last));
    s_last_tx_tick = 0U;
    s_seq = 0U;
}

void BLE_Stream_SetMPU6050(const MPU6050_data_out *d)
{
    if (d != NULL) {
        s_mpu_last = *d;
    }
}

static void pack_u8(uint8_t **p, uint8_t v)
{
    **p = v;
    *p += 1;
}

static void pack_u32(uint8_t **p, uint32_t v)
{
    memcpy(*p, &v, sizeof(v));
    *p += sizeof(v);
}

static void pack_f32(uint8_t **p, float v)
{
    memcpy(*p, &v, sizeof(v));
    *p += sizeof(v);
}

void BLE_Stream_Process(void)
{
    if (!APP_FLAG(CONNECTED) || !APP_FLAG(NOTIFICATIONS_ENABLED)) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - s_last_tx_tick) < BLE_STREAM_PERIOD_MS) {
        return;
    }

    float ypr[3];
    MEMS_Kinematics_GetEuler(ypr);

    float acc_arm[3];
    MEMS_Kinematics_GetAccel(acc_arm);
    const float acc_palm[3] = { s_mpu_last.ax, s_mpu_last.ay, s_mpu_last.az };
    /* [REVIEW-FIX:7] se l'MPU del palmo non è sano (init fallita o 3+ letture
     * KO consecutive) trasmettiamo NaN invece di un valore stantio o spurio.
     * Il reader stampa "nan" in colonna CSV e l'utente vede subito il problema.
     * Per ripristinare il vecchio comportamento (sempre un float plausibile)
     * basta togliere il controllo e tenere solo il ramo ArmPalm_RelativePitchDeg. */
    float palm_rel_deg;
    if (MPU6050_IsHealthy()) {
        palm_rel_deg = ArmPalm_RelativePitchDeg(acc_arm, acc_palm);
    } else {
        palm_rel_deg = NAN;
    }

    uint8_t close_pinza=get_pinza_position();

    uint8_t buf[BLE_STREAM_PAYLOAD_LEN];
    uint8_t *p = buf;

    pack_u8(&p, BLE_STREAM_MAGIC);
    pack_u8(&p, s_seq);
    pack_u32(&p, now);
    pack_f32(&p, ypr[0]);
    pack_f32(&p, ypr[1]);
    pack_f32(&p, ypr[2]);
    pack_f32(&p, palm_rel_deg);
    pack_u8(&p, close_pinza);

    tBleStatus ret = aci_gatt_update_char_value_ext(connection_handle,
                                                    sampleServHandle,
                                                    TXCharHandle,
                                                    1,
                                                    BLE_STREAM_PAYLOAD_LEN,
                                                    0,
                                                    BLE_STREAM_PAYLOAD_LEN,
                                                    buf);

    if (ret == BLE_STATUS_SUCCESS) {
        s_seq++;
        s_last_tx_tick = now;
    } else if (ret == BLE_STATUS_INSUFFICIENT_RESOURCES) {
        /* Buffer radio pieno: salta questo frame, riprova al prossimo tick. */
        APP_FLAG_SET(TX_BUFFER_FULL);
    } else {
        /* Errore non recuperabile su questo frame: avanza comunque il tick
         * per non saturare la CPU con retry immediati. */
        s_last_tx_tick = now;
    }
}
