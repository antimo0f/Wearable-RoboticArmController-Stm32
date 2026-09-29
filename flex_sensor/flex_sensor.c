#include "flex_sensor.h"
#include "main.h"   /* per FLEX_SNS_PWR_Pin, FLEX_SNS_PWR_GPIO_Port */

static ADC_HandleTypeDef *s_hadc      = NULL;
static FlexSensor_Data    s_data      = {0};
static uint16_t           s_buf[FLEX_AVG_SAMPLES] = {0};
static uint8_t            s_idx       = 0;
static uint32_t           s_sum       = 0;
static uint8_t            s_filled    = 0;  /* buffer riempito almeno una volta */

void FlexSensor_PowerOn(void)
{
    HAL_GPIO_WritePin(FLEX_SNS_PWR_GPIO_Port, FLEX_SNS_PWR_Pin, GPIO_PIN_SET);
}

void FlexSensor_PowerOff(void)
{
    HAL_GPIO_WritePin(FLEX_SNS_PWR_GPIO_Port, FLEX_SNS_PWR_Pin, GPIO_PIN_RESET);
}

static int read_one_sample(uint16_t *out)
{
    if (HAL_ADC_Start(s_hadc) != HAL_OK) {
        FLEX_DBG_ERR("HAL_ADC_Start failed");
        return -1;
    }
    if (HAL_ADC_PollForConversion(s_hadc, 10U) != HAL_OK) {
        HAL_ADC_Stop(s_hadc);
        FLEX_DBG_ERR("HAL_ADC_PollForConversion timeout");
        return -2;
    }
    *out = (uint16_t)HAL_ADC_GetValue(s_hadc);
    HAL_ADC_Stop(s_hadc);
    FLEX_DBG_TRACE("sample raw=%u", (unsigned)*out);
    return 0;
}

int FlexSensor_Init(ADC_HandleTypeDef *hadc)
{
    FLEX_DBG_BOOT("Init: hadc=%p, avg=%u, Rpd=%u Ohm, Vref=%u mV",
                  (void*)hadc, (unsigned)FLEX_AVG_SAMPLES,
                  (unsigned)FLEX_RPD_OHM, (unsigned)FLEX_VREF_MV);
    if (hadc == NULL) {
        FLEX_DBG_ERR("Init: hadc NULL");
        return -1;
    }
    s_hadc   = hadc;
    s_idx    = 0;
    s_sum    = 0;
    s_filled = 0;
    for (uint8_t i = 0; i < FLEX_AVG_SAMPLES; i++) {
        s_buf[i] = 0;
    }

    FlexSensor_PowerOn();
    FLEX_DBG_BOOT("PowerOn (PC3=HIGH), warm-up...");
    HAL_Delay(2);  /* settling tempo di salita partitore */

    /* warm-up: riempi il buffer con campioni reali */
    for (uint8_t i = 0; i < FLEX_AVG_SAMPLES; i++) {
        uint16_t v = 0;
        if (read_one_sample(&v) != 0) {
            FLEX_DBG_ERR("Init: warm-up sample %u failed", (unsigned)i);
            return -2;
        }
        s_buf[i] = v;
        s_sum   += v;
    }
    s_filled = 1;

    s_data.raw        = s_buf[FLEX_AVG_SAMPLES - 1];
    s_data.filtered   = (uint16_t)(s_sum / FLEX_AVG_SAMPLES);
    s_data.millivolts = (uint16_t)(((uint32_t)s_data.filtered * FLEX_VREF_MV) / FLEX_ADC_FULLSCALE);
    s_data.resistance = (s_data.millivolts > 0U)
        ? ((uint32_t)FLEX_RPD_OHM * (FLEX_VREF_MV - s_data.millivolts)) / s_data.millivolts
        : 0xFFFFFFFFU;
    s_data.touched = (s_data.millivolts > FLEX_TOUCH_THRESHOLD_MV) ? 1U : 0U;
    FLEX_DBG_BOOT("Init OK: filtered=%u mV=%u R=%lu touched=%u",
                  (unsigned)s_data.filtered,
                  (unsigned)s_data.millivolts,
                  (unsigned long)s_data.resistance,
                  (unsigned)s_data.touched);
    return 0;
}

int FlexSensor_Process(void)
{
    if (s_hadc == NULL) {
        return -1;
    }
    uint16_t v = 0;
    if (read_one_sample(&v) != 0) {
        return -2;
    }

    /* aggiorna media mobile (somma incrementale) */
    s_sum     -= s_buf[s_idx];
    s_buf[s_idx] = v;
    s_sum     += v;
    s_idx      = (uint8_t)((s_idx + 1U) % FLEX_AVG_SAMPLES);

    s_data.raw        = v;
    s_data.filtered   = (uint16_t)(s_sum / FLEX_AVG_SAMPLES);
    s_data.millivolts = (uint16_t)(((uint32_t)s_data.filtered * FLEX_VREF_MV) / FLEX_ADC_FULLSCALE);

    if (s_data.millivolts == 0U) {
        s_data.resistance = 0xFFFFFFFFU;  /* sensore aperto / nessun contatto */
    } else if (s_data.millivolts >= FLEX_VREF_MV) {
        s_data.resistance = 0U;           /* sensore in corto */
    } else {
        s_data.resistance =
            ((uint32_t)FLEX_RPD_OHM * (FLEX_VREF_MV - s_data.millivolts)) / s_data.millivolts;
    }

    uint8_t prev_touched = s_data.touched;
    s_data.touched = (s_data.millivolts > FLEX_TOUCH_THRESHOLD_MV) ? 1U : 0U;

    /* Livello 2: notifica solo le transizioni touched <-> non touched */
    if (s_data.touched != prev_touched) {
        if (s_data.touched) {
            FLEX_DBG_EVT("TOUCH ON  (mV=%u R=%lu)",
                         (unsigned)s_data.millivolts,
                         (unsigned long)s_data.resistance);
        } else {
            FLEX_DBG_EVT("TOUCH OFF (mV=%u R=%lu)",
                         (unsigned)s_data.millivolts,
                         (unsigned long)s_data.resistance);
        }
    }

    /* Livello 3: dump periodico (ogni FLEX_DBG_PERIODIC_EVERY chiamate). */
    static uint32_t dbg_cnt = 0;
    dbg_cnt++;
    if ((dbg_cnt % FLEX_DBG_PERIODIC_EVERY) == 0U) {
        FLEX_DBG_PERIODIC("raw=%u filt=%u mV=%u R=%lu touched=%u",
                          (unsigned)s_data.raw,
                          (unsigned)s_data.filtered,
                          (unsigned)s_data.millivolts,
                          (unsigned long)s_data.resistance,
                          (unsigned)s_data.touched);
    }
    return 0;
}

const FlexSensor_Data *FlexSensor_Get(void)
{
    return &s_data;
}

/* Lettura "cruda" media: utility privata per il self-test, non aggiorna lo stato.
 * Fa 16 conversioni e ritorna la media in mV. */
static uint16_t selftest_read_mv(void)
{
    uint32_t sum = 0;
    uint8_t  n   = 0;
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t v = 0;
        if (read_one_sample(&v) == 0) {
            sum += v;
            n++;
        }
    }
    if (n == 0) return 0;
    uint32_t avg = sum / n;
    return (uint16_t)((avg * FLEX_VREF_MV) / FLEX_ADC_FULLSCALE);
}

void FlexSensor_SelfTest(void)
{
    if (s_hadc == NULL) {
        printf("[FLEX][TEST] NO INIT - chiamare FlexSensor_Init prima\r\n");
        return;
    }

    printf("[FLEX][TEST] === Inizio self-test cablaggio ===\r\n");

    /* Fase 1: PC3 LOW. Atteso V_PC2 ~ 0 (solo R_pd attivo verso GND). */
    FlexSensor_PowerOff();
    HAL_Delay(50);
    uint16_t mv_low = selftest_read_mv();
    printf("[FLEX][TEST] PC3=LOW  -> V_PC2 = %u mV (atteso ~0)\r\n", (unsigned)mv_low);

    /* Fase 2: PC3 HIGH. Atteso V_PC2 dipendente da R_fsr. */
    FlexSensor_PowerOn();
    HAL_Delay(50);
    uint16_t mv_high = selftest_read_mv();
    printf("[FLEX][TEST] PC3=HIGH -> V_PC2 = %u mV (atteso 30..3000 a seconda di R_fsr)\r\n",
           (unsigned)mv_high);

    int32_t delta = (int32_t)mv_high - (int32_t)mv_low;
    printf("[FLEX][TEST] delta = %ld mV\r\n", (long)delta);

    /* Interpretazione automatica */
    printf("[FLEX][TEST] --- Diagnosi ---\r\n");

    if (mv_low > 200) {
        printf("[FLEX][TEST] !!! mv_low alto: pull-down R_pd staccato da GND,\r\n"
               "             oppure PC2 non collegato al partitore (filo via).\r\n"
               "             PC2 sta floating o vede un'altra tensione.\r\n");
    } else {
        printf("[FLEX][TEST] OK  mv_low basso: R_pd verso GND sembra connesso.\r\n");
    }

    if (delta < 50) {
        printf("[FLEX][TEST] !!! delta troppo piccolo: PC3 non sta variando V_PC2.\r\n"
               "             Possibili cause:\r\n"
               "               - filo da PC3 al sensore staccato\r\n"
               "               - filo dal sensore a PC2 staccato\r\n"
               "               - sensore morto (resistenza fissa molto alta)\r\n"
               "               - PC3 non riesce a pilotare HIGH (bug GPIO)\r\n");
    } else if (delta > 2500) {
        printf("[FLEX][TEST] OK  delta ampio: PC3 pilota correttamente,\r\n"
               "             il sensore presenta bassa resistenza (premuto?).\r\n");
    } else {
        printf("[FLEX][TEST] OK  delta in range medio: cablaggio corretto,\r\n"
               "             la striscia presenta resistenza intermedia.\r\n");
    }

    /* Stima V_PC3 effettiva nel caso di cortocircuito (R_fsr=0): V_PC2 = V_PC3.
     * Se sospetti corto, mv_high dovrebbe essere ~3300. */
    printf("[FLEX][TEST] Se hai cortocircuitato il sensore con un filo:\r\n"
           "             mv_high atteso ~3300 mV. Letto %u mV.\r\n",
           (unsigned)mv_high);
    if (mv_high < 2900 && mv_high > 200) {
        printf("[FLEX][TEST]    -> probabile: PC3 NON arriva a 3.3 V\r\n"
               "                  (filo PC3 staccato dal sensore, o il sensore\r\n"
               "                   non e' davvero in corto).\r\n");
    }

    /* ----------------------------------------------------------------
     * Diagnostica register-level: leggo direttamente GPIOC->MODER/ODR/IDR.
     * Salta del tutto HAL: se questi numeri sono sbagliati il bug e' qui.
     * MODER (2 bit per pin):  00=INPUT 01=OUTPUT 10=ALTFUNC 11=ANALOG
     * ODR/IDR: 1 bit per pin, valori 0/1.
     * ---------------------------------------------------------------- */
    uint32_t moder = GPIOC->MODER;
    uint32_t odr   = GPIOC->ODR;
    uint32_t idr   = GPIOC->IDR;
    uint8_t  pc2_mode = (moder >> (2 * 2)) & 0x3U;  /* bit [5:4] */
    uint8_t  pc3_mode = (moder >> (3 * 2)) & 0x3U;  /* bit [7:6] */
    uint8_t  pc2_idr  = (idr   >> 2) & 0x1U;
    uint8_t  pc3_idr  = (idr   >> 3) & 0x1U;
    uint8_t  pc3_odr  = (odr   >> 3) & 0x1U;
    static const char *MODE_STR[4] = {"INPUT", "OUTPUT", "ALTFUNC", "ANALOG"};
    printf("[FLEX][TEST] GPIOC->MODER = 0x%08lX\r\n", (unsigned long)moder);
    printf("[FLEX][TEST]   PC2 mode = %u (%s)  atteso 3 (ANALOG)\r\n",
           pc2_mode, MODE_STR[pc2_mode]);
    printf("[FLEX][TEST]   PC3 mode = %u (%s)  atteso 1 (OUTPUT)\r\n",
           pc3_mode, MODE_STR[pc3_mode]);
    printf("[FLEX][TEST] GPIOC->ODR  PC3 = %u  (1 = sto pilotando HIGH)\r\n", pc3_odr);
    printf("[FLEX][TEST] GPIOC->IDR  PC2 = %u  PC3 = %u  (livello fisico ai pin)\r\n",
           pc2_idr, pc3_idr);

    /* Test forzato: scrittura LOW poi HIGH direttamente su BSRR,
     * lettura immediata dell'IDR per vedere se i pin reagiscono fisicamente. */
    GPIOC->BSRR = (1U << (3 + 16));  /* BR3: reset PC3 a 0 */
    for (volatile int i = 0; i < 1000; i++) {}
    uint8_t idr_after_low  = (GPIOC->IDR >> 3) & 0x1U;
    GPIOC->BSRR = (1U << 3);         /* BS3: set PC3 a 1 */
    for (volatile int i = 0; i < 1000; i++) {}
    uint8_t idr_after_high = (GPIOC->IDR >> 3) & 0x1U;
    printf("[FLEX][TEST] BSRR test: PC3 letto %u dopo BR3, %u dopo BS3\r\n",
           idr_after_low, idr_after_high);
    if (idr_after_low == 0 && idr_after_high == 1) {
        printf("[FLEX][TEST]   OK: PC3 segue ODR -> il pin del Nucleo funziona.\r\n");
    } else {
        printf("[FLEX][TEST]   !!! PC3 NON segue ODR -> pin fisicamente bloccato o\r\n"
               "                                       configurato male in MODER.\r\n");
    }

    printf("[FLEX][TEST] === Fine self-test ===\r\n");

    /* Lascia PC3 a HIGH per il normale funzionamento */
    FlexSensor_PowerOn();
    HAL_Delay(5);
}
