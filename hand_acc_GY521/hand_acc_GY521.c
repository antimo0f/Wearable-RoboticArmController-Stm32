
#include "hand_acc_GY521.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>


extern I2C_HandleTypeDef hi2c3;

/* [REVIEW-FIX:2] Error_Handler() non più chiamato qui (vedi MPU6050_Init).
 * Riattivare l'include solo se si vuole tornare al comportamento "fatal". */
/* extern void Error_Handler(void); */

/* [REVIEW-FIX:10] tutti i magic number del MPU6050 estratti in #define.
 * Centralizzare facilita un eventuale port a un altro accelerometro o cambio
 * di indirizzo (es. AD0=1 → 0x69). */
#define MPU6050_I2C_ADDR_7BIT   0x68U
#define MPU6050_I2C_ADDR        (MPU6050_I2C_ADDR_7BIT << 1)
#define MPU6050_REG_SMPLRT_DIV  0x19U
#define MPU6050_REG_GYRO_CONFIG 0x1BU
#define MPU6050_REG_ACCEL_CFG   0x1CU
#define MPU6050_REG_ACCEL_XOUT  0x3BU
#define MPU6050_REG_PWR_MGMT_1  0x6BU
#define MPU6050_REG_WHO_AM_I    0x75U
#define MPU6050_WHO_AM_I_VAL    0x68U
/* Sensibilità: ±2g → 16384 LSB/g; temp: scale 340, offset 36.53. */
#define MPU6050_ACC_LSB_PER_G   16384.0f
#define MPU6050_TEMP_SCALE      340.0f
#define MPU6050_TEMP_OFFSET     36.53f

/* [REVIEW-FIX:4] timeout finiti al posto di HAL_MAX_DELAY.
 * Con HAL_MAX_DELAY un I2C bloccato (es. linee SDA/SCL basse per ESD) congelava
 * il main loop per sempre. 50 ms è ampiamente sufficiente per le transazioni
 * MPU6050 a 400 kHz (~0.5 ms reali). Aumentare se l'I2C è condiviso con
 * device lenti. */
#define MPU6050_I2C_TIMEOUT_MS  50U

/* Filtro passa-basso EMA sulle assi accelerometro:
 *   s_n = alpha * x_n + (1 - alpha) * s_{n-1}
 * alpha basso = più morbido, 1.0 = filtro disattivato.
 *
 * [REVIEW-FIX:11] NOTA: questo α è dipendente dalla frequenza di chiamata di
 * MPU6050_Read. Se la cadenza cambia (es. main loop più veloce/lento) cambia
 * anche la frequenza di taglio effettiva. Per renderlo invariante andrebbe
 * ricavato da una fc target e dal dt misurato:
 *     alpha = dt / (RC + dt), con RC = 1/(2*pi*fc).
 * Lasciato fisso per ora per non cambiare il comportamento del filtro. */
#define MPU6050_ACC_ALPHA   0.1f
static float ax_filt = 0.0f, ay_filt = 0.0f, az_filt = 0.0f;
static uint8_t filt_init = 0U;

/* [REVIEW-FIX:7] flag interno di salute del sensore. Si abbassa se MPU6050_Init
 * fallisce o se 3 letture I2C consecutive ritornano errore (vedi MPU6050_Read). */
static uint8_t s_healthy = 0U;
static uint8_t s_consec_errors = 0U;
#define MPU6050_MAX_CONSEC_ERR  3U

/* [REVIEW-FIX:3] stampa di debug a 50+ Hz spostata dietro flag. Era il principale
 * carico UART del progetto. Definire MPU6050_VERBOSE=1 per riattivarla.
 *
 * [DEAD-CODE:H1] !!! BUG ATTIVO !!! Il default era stato lasciato a 1 nonostante
 * il commento di REVIEW-FIX:3 dica il contrario. MPU6050_Read viene chiamato a
 * OGNI iterazione del main loop (non rate-limited), e per ciascuna chiamata
 * questa printf formattava ~60 byte + li mandava via __io_putchar che è
 * blocking su UART2 con timeout 5 ms.
 * A 921600 baud sono ~0.65 ms per stampa, ma è la formattazione `%6.3f`
 * (4× softfloat) il vero costo: lo F401RE non ha FPU per double, quindi
 * ogni `%f` invoca __aeabi_dadd/dmul. Risultato: main loop strozzato a
 * ~1-2 kHz invece di > 10 kHz, con ripercussioni sulla cadenza MotionFX
 * e BLE stream. Default ora 0 (silenzioso). Per debug ad-hoc: settare a 1
 * SOLO in fase di porting o calibrazione. */
#ifndef MPU6050_VERBOSE
#define MPU6050_VERBOSE 0
#endif

uint8_t MPU6050_Init(void) {
    uint8_t data;
    HAL_StatusTypeDef st;

    // Wake up + clock PLL giroscopio
    data = 0x01;
    st = HAL_I2C_Mem_Write(&hi2c3, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, 1,
                           &data, 1, MPU6050_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        printf("[MPU6050] init FAIL @PWR_MGMT_1 hal=%d (slave non risponde su I2C3)\r\n", (int)st);
        s_healthy = 0U; return 1U;
    }

    // Accelerometro ±2g
    data = 0x00;
    st = HAL_I2C_Mem_Write(&hi2c3, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CFG, 1,
                           &data, 1, MPU6050_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        printf("[MPU6050] init FAIL @ACCEL_CFG hal=%d\r\n", (int)st);
        s_healthy = 0U; return 1U;
    }

    // Giroscopio ±250°/s
    data = 0x00;
    st = HAL_I2C_Mem_Write(&hi2c3, MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, 1,
                           &data, 1, MPU6050_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        printf("[MPU6050] init FAIL @GYRO_CONFIG hal=%d\r\n", (int)st);
        s_healthy = 0U; return 1U;
    }

    // Verifica connessione (opzionale, solo per debug)
    uint8_t who = 0;
    st = HAL_I2C_Mem_Read(&hi2c3, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, 1,
                          &who, 1, MPU6050_I2C_TIMEOUT_MS);
    /* [REVIEW-FIX:2] PRIMA: if (who != 0x68) Error_Handler();  che disabilitava
     * IRQ e bloccava per sempre — fatale per un wearable BLE. ORA: segnaliamo
     * lo stato e lasciamo decidere al chiamante. Il main può continuare senza
     * il palmo (lo stream BLE setterà palm_rel_deg a NaN, vedi ble_stream.c). */
    if (st != HAL_OK || who != MPU6050_WHO_AM_I_VAL) {
        printf("[MPU6050] init FAIL  hal=%d who=0x%02X\r\n", (int)st, who);
        s_healthy = 0U;
        return 1U;
    }

    s_healthy       = 1U;
    s_consec_errors = 0U;
    filt_init       = 0U;
    return 0U;
}

uint8_t MPU6050_IsHealthy(void) { return s_healthy; }

// Nel while(1), OGNI VOLTA che vuoi i dati
uint8_t MPU6050_Read(MPU6050_data_out* data_out) {
    uint8_t buf[14];

    /* [REVIEW-FIX:4] timeout finito. Se l'I2C si pianta NON congeliamo il main.
     * [REVIEW-FIX:7] errore → s_consec_errors++. Dopo MPU6050_MAX_CONSEC_ERR
     * letture fallite di fila marchiamo il sensore come non sano: il main /
     * BLE_Stream può così evitare di pubblicare dati stantii. */
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c3, MPU6050_I2C_ADDR,
                                            MPU6050_REG_ACCEL_XOUT, 1,
                                            buf, 14, MPU6050_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        if (s_consec_errors < 255U) s_consec_errors++;
        if (s_consec_errors >= MPU6050_MAX_CONSEC_ERR) {
            s_healthy = 0U;
        }
        return 1U;
    }
    s_consec_errors = 0U;
    s_healthy = 1U;

    float ax_raw = (int16_t)(buf[0]  << 8 | buf[1]) / MPU6050_ACC_LSB_PER_G;
    float ay_raw = (int16_t)(buf[2]  << 8 | buf[3]) / MPU6050_ACC_LSB_PER_G;
    float az_raw = (int16_t)(buf[4]  << 8 | buf[5]) / MPU6050_ACC_LSB_PER_G;

    if (!filt_init) {
        ax_filt = ax_raw;
        ay_filt = ay_raw;
        az_filt = az_raw;
        filt_init = 1U;
    } else {
        ax_filt += MPU6050_ACC_ALPHA * (ax_raw - ax_filt);
        ay_filt += MPU6050_ACC_ALPHA * (ay_raw - ay_filt);
        az_filt += MPU6050_ACC_ALPHA * (az_raw - az_filt);
    }

    data_out->ax   = ax_filt;
    data_out->ay   = ay_filt;
    data_out->az   = az_filt;
    /* [DEAD-CODE:H2] data_out->temp: campo della struct mai letto a runtime
     * (lo stream BLE usa solo ax/ay/az, vedi BLE_Stream_Process in ble_stream.c).
     * La divisione + offset è un floating-point sprecato. Mantengo l'assegnazione
     * (1 div + 1 add, ~10 cicli FPU) per non rompere il significato semantico
     * della struct, ma il campo si può rimuovere dal header se non serve.
     * NB: i byte buf[6]/buf[7] (TEMP_OUT) sono letti comunque dall'I2C burst di
     * 14 byte; ridurre la lettura a 6 byte (solo accel) farebbe risparmiare
     * ~200 µs per Read sull'I2C 400 kHz. */
    data_out->temp = (int16_t)(buf[6] << 8 | buf[7]) / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;

    /* [REVIEW-FIX:3] stampa attiva solo con MPU6050_VERBOSE=1. La call originale
     * costava ~70 byte di printf a ~50 Hz tramite __io_putchar bloccante. */
#if MPU6050_VERBOSE
    printf("6050-AX: %6.3f g  AY: %6.3f g  AZ: %6.3f g  TEMP: %5.2f C\r\n",
           data_out->ax, data_out->ay, data_out->az, data_out->temp);
#endif
    return 0U;
}
