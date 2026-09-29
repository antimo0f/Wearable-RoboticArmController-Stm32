#ifndef FLEX_SENSOR_H_
#define FLEX_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* =====================================================================
 *  DEBUG PRINT — attivabile con macro
 * =====================================================================
 *   FLEX_DBG_LEVEL = 0  -> nessun print  (zero overhead, le printf sono compilate via)
 *                  = 1  -> solo errori e boot
 *                  = 2  -> + eventi (touched on/off, transizioni)
 *                  = 3  -> + dump periodico del campione (raw, mV, R, touched)
 *                  = 4  -> + dump di OGNI conversione (molto verboso, attenzione UART)
 *
 * Si puo' sovrascrivere da compilatore (-DFLEX_DBG_LEVEL=3) o cambiare qui sotto.
 * Il prefisso "[FLEX]" identifica i messaggi sul terminale UART.
 * --------------------------------------------------------------------- */
#ifndef FLEX_DBG_LEVEL
#define FLEX_DBG_LEVEL 3
#endif

/* Periodo (in numero di chiamate a FlexSensor_Process) per il dump livello 3.
 * A loop tipico ~1 kHz, 200 = un print ogni ~200 ms. */
#ifndef FLEX_DBG_PERIODIC_EVERY
#define FLEX_DBG_PERIODIC_EVERY 200U
#endif

#include <stdio.h>  /* serve solo se i livelli sono attivi, ma e' gia' incluso ovunque */

#if (FLEX_DBG_LEVEL >= 1)
  #define FLEX_DBG_ERR(fmt, ...)   printf("[FLEX][ERR] "  fmt "\r\n", ##__VA_ARGS__)
  #define FLEX_DBG_BOOT(fmt, ...)  printf("[FLEX][BOOT] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define FLEX_DBG_ERR(fmt, ...)   ((void)0)
  #define FLEX_DBG_BOOT(fmt, ...)  ((void)0)
#endif

#if (FLEX_DBG_LEVEL >= 2)
  #define FLEX_DBG_EVT(fmt, ...)   printf("[FLEX][EVT] "  fmt "\r\n", ##__VA_ARGS__)
#else
  #define FLEX_DBG_EVT(fmt, ...)   ((void)0)
#endif

#if (FLEX_DBG_LEVEL >= 3)
  #define FLEX_DBG_PERIODIC(fmt, ...)  printf("[FLEX] "   fmt "\r\n", ##__VA_ARGS__)
#else
  #define FLEX_DBG_PERIODIC(fmt, ...)  ((void)0)
#endif

#if (FLEX_DBG_LEVEL >= 4)
  #define FLEX_DBG_TRACE(fmt, ...)     printf("[FLEX][T] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define FLEX_DBG_TRACE(fmt, ...)     ((void)0)
#endif

/**
 * Driver per striscia di pressione resistiva ZD10-100.
 *
 * Hardware:
 *   - Sensore (FSR) tra PC3 (alimentazione gated, GPIO_OUTPUT) e PC2.
 *   - Resistenza di pull-down R_pd = 10 kOhm tra PC2 e GND.
 *   - PC2 = ADC1_IN12, già configurato da CubeMX.
 *
 * Modello partitore:
 *   V_adc = V_cc * R_pd / (R_fsr + R_pd)
 *   R_fsr = R_pd * (V_cc - V_adc) / V_adc
 *
 * Filtraggio: media mobile in software (no condensatore sul nodo ADC).
 */

#define FLEX_AVG_SAMPLES   8U      /* finestra media mobile (potenza di 2) */
#define FLEX_VREF_MV       3300U   /* tensione di riferimento ADC, mV */
#define FLEX_ADC_FULLSCALE 4095U   /* 12 bit */
#define FLEX_RPD_OHM       10000U  /* resistenza di pull-down */

/* Soglia di tocco (V_adc): 2.90 V corrisponde a R_fsr ~ 1.4 kOhm, cioe' una
 * pressione decisa (vicina al fondo scala dei 500 g). Sotto 2.9 V -> touched=0,
 * sopra 2.9 V -> touched=1. Tarato sperimentalmente. */
#define FLEX_TOUCH_THRESHOLD_MV  1500U

typedef struct {
    uint16_t raw;          /* ultima conversione 12-bit grezza        */
    uint16_t filtered;     /* media mobile su FLEX_AVG_SAMPLES         */
    uint16_t millivolts;   /* tensione filtrata in mV                  */
    uint32_t resistance;   /* R_fsr stimata in Ohm                     */
    uint8_t  touched;      /* 1 se filtered > soglia, 0 altrimenti     */
} FlexSensor_Data;

/**
 * Inizializza il driver. Accende l'alimentazione del sensore (PC3 = HIGH),
 * salva l'handle ADC, azzera il buffer di media mobile e fa qualche
 * conversione di "warm-up" per stabilizzare il filtro.
 *
 * @param hadc Handle ADC1 (canale 12 già configurato da MX_ADC1_Init).
 * @return 0 se ok, !=0 in caso di errore HAL.
 */
int FlexSensor_Init(ADC_HandleTypeDef *hadc);

/**
 * Esegue una conversione, aggiorna media mobile e flag touched.
 * Non bloccante in senso "lungo" (tipico < 1 ms a 480 cicli su ADC clock 21 MHz).
 * Chiamare nel main loop alla frequenza desiderata (es. 100-1000 Hz).
 *
 * @return 0 se ok, !=0 se conversione fallita (lo stato precedente resta valido).
 */
int FlexSensor_Process(void);

/** Accesso ai dati filtrati piu' recenti. Sempre valido dopo Init(). */
const FlexSensor_Data *FlexSensor_Get(void);

/** Accende/spegne l'alimentazione del sensore (power gating per risparmio). */
void FlexSensor_PowerOn(void);
void FlexSensor_PowerOff(void);

/**
 * Test diagnostico cablaggio (chiamare DOPO FlexSensor_Init).
 * Pilota PC3 LOW e HIGH, legge l'ADC, stampa interpretazione:
 *   - V_low  -> tensione su PC2 con PC3=LOW. Atteso ~0 mV se R_pd OK.
 *   - V_high -> tensione su PC2 con PC3=HIGH. Atteso 0..3300 mV a seconda di R_fsr.
 *   - delta  -> V_high - V_low. Se vicino a 0 -> sensore o PC3 scollegati.
 * Lascia PC3 a HIGH al ritorno.
 */
void FlexSensor_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* FLEX_SENSOR_H_ */
