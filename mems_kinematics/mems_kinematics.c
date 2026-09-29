/**
  ******************************************************************************
  * @file    mems_kinematics.c
  * @brief   Implementazione della libreria per il calcolo dell'orientamento 
  *          tramite MotionFX e l'applicazione a punti 3D.
  ******************************************************************************
  */
#include "mems_kinematics.h"
#include "iks4a1_mems_control.h"
#include "stm32f4xx_hal.h"
#include "motion_fx.h"
#include <stdio.h>

/* [REVIEW-FIX:3] tutti i printf periodici (Yaw/Pitch/Roll, Heading, Quaternion) sono
 * ora gated dietro questa macro. Lasciati attivi a 100 Hz saturavano UART2 e con
 * __io_putchar bloccante (HAL_MAX_DELAY) facevano slittare il TIM1 → degradavano
 * MotionFX e lo stream BLE a 50 Hz. Per riattivare le stampe definire
 * MEMS_KINEMATICS_VERBOSE=1 (build flag o qui sotto). */
#ifndef MEMS_KINEMATICS_VERBOSE
#define MEMS_KINEMATICS_VERBOSE 0
#endif

/* --- Private Defines --- */
#define MFX_STATE_SIZE_INTERNAL 2432
#define DELTA_TIME_S            0.01f  /* 100 Hz = 0.01 secondi */
#define ALGO_FREQ 				 100U /* Algorithm frequency 100Hz */
#define ACC_ODR  				((float)ALGO_FREQ)
#define ACC_FS  				4 /* FS = <-4g, 4g> */

/* --- Private Variables --- */
/* Buffer allineato a 4 byte per l'algoritmo MotionFX */
static uint32_t mfx_state_buffer[MFX_STATE_SIZE_INTERNAL / 4];
static MFXState_t mfx_state;

static MFX_knobs_t knobs;
static MFX_input_t data_in;
static MFX_output_t data_out;
extern TIM_HandleTypeDef htim1;
/* [REVIEW-FIX:1] aggiunto 'volatile'. datareq è scritto in HAL_TIM_PeriodElapsedCallback
 * (contesto ISR) e letto nel main loop. Senza 'volatile', a -O2/-O3 il compilatore può
 * cachare il valore in un registro e il check `if(datareq==1U)` non vedere mai
 * l'aggiornamento dell'ISR. Rimuovere volatile solo se si toglie la lettura/scrittura
 * cross-context. */
static volatile int datareq = 0;
/* [REVIEW-FIX:6] timestamp dell'ultimo Update, in ms (HAL_GetTick).
 * Serve per passare a MotionFX il dt REALE invece del nominale DELTA_TIME_S.
 * Se il main loop salta un tick TIM1 (es. per un printf bloccante), DELTA_TIME_S
 * fisso introduce drift nell'integrazione di yaw/heading. Per disattivare e
 * tornare al comportamento originale, vedere MEMS_Kinematics_Update. */
static uint32_t s_last_update_tick = 0U;
/* --- Public Functions --- */

static void Init_Sensors(void)
{
  BSP_SENSOR_ACC_Init();
  BSP_SENSOR_GYR_Init();
  BSP_SENSOR_MAG_Init();
  /* [DEAD-CODE:K1] RIPRISTINATO — init PRESS/TEMP/HUM riattivate per sicurezza. */
  BSP_SENSOR_PRESS_Init();
  BSP_SENSOR_TEMP_Init();
  BSP_SENSOR_HUM_Init();
  BSP_SENSOR_ACC_SetOutputDataRate(ACC_ODR);
  BSP_SENSOR_ACC_SetFullScale(ACC_FS);
}

void MEMS_Kinematics_Init(void)
{

	Init_Sensors();
    /* 1. Abilitazione sensori MEMS fisici */
    BSP_SENSOR_ACC_Enable();
    BSP_SENSOR_GYR_Enable();
    BSP_SENSOR_MAG_Enable();

    /* [REVIEW-FIX:3] stampa di boot mantenuta (one-shot, non in hot path). */
    printf("Inizializzazione OK \r\n");
    /* 2. Inizializzazione di MotionFX */
    mfx_state = (MFXState_t)mfx_state_buffer;
    MotionFX_initialize(mfx_state);

    /* 3. Configurazione dei parametri interni (Knobs) */
    MotionFX_getKnobs(mfx_state, &knobs);
    knobs.ATime = DELTA_TIME_S; 
    MotionFX_setKnobs(mfx_state, &knobs);

    /* 4. Attivazione del motore a 9-assi (Accelerometro + Giroscopio + Magnetometro) */
    MotionFX_enable_9X(mfx_state, MFX_ENGINE_ENABLE);


    HAL_TIM_Base_Start_IT(&htim1);
    printf("Inizializzazione TIM1  OK");
}


void MEMS_Kinematics_process(void)
{
	if(datareq==1U){

		datareq=0U;
		MEMS_Kinematics_Update();
		/* [REVIEW-FIX:3] le stampe a 100 Hz erano la principale fonte di latenza nel
		 * main loop. Ora attive solo se MEMS_KINEMATICS_VERBOSE != 0. */
#if MEMS_KINEMATICS_VERBOSE
		MEMS_Kinematics_printRotation();
		MEMS_Kinematics_printHeading();
#endif
	}


}


void MEMS_Kinematics_Update(void)
{
    IKS4A1_MOTION_SENSOR_Axes_t acc, gyr, mag;
    /* [REVIEW-FIX:6] dt misurato in ms (risoluzione 1 ms via HAL_GetTick) e
     * convertito in secondi. Clampato a [2ms, 50ms] per evitare:
     *   - dt=0 al primo Update (s_last_update_tick ancora 0)
     *   - dt enormi dopo lunghi blocchi che farebbero "saltare" l'algoritmo.
     * Per tornare al comportamento originale (dt nominale fisso), riassegnare
     * delta_time = DELTA_TIME_S e rimuovere il blocco sotto. */
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = (s_last_update_tick == 0U) ? 10U : (now - s_last_update_tick);
    if (dt_ms < 2U)  dt_ms = 2U;
    if (dt_ms > 50U) dt_ms = 50U;
    s_last_update_tick = now;
    float delta_time = (float)dt_ms * 0.001f;
    /* Lettura grezza dai sensori */
    BSP_SENSOR_ACC_GetAxes(&acc);
    BSP_SENSOR_GYR_GetAxes(&gyr);
    BSP_SENSOR_MAG_GetAxes(&mag);

    /* Conversione nelle unità di misura richieste da MotionFX */
    data_in.acc[0] = acc.x * 0.001f; /* da mg a g */
    data_in.acc[1] = acc.y * 0.001f;
    data_in.acc[2] = acc.z * 0.001f;

    data_in.gyro[0] = gyr.x * 0.001f; /* da mdps a dps */
    data_in.gyro[1] = gyr.y * 0.001f;
    data_in.gyro[2] = gyr.z * 0.001f;

    data_in.mag[0] = mag.x * (0.1f / 50.0f); /* da mGauss a uT / 50 */
    data_in.mag[1] = mag.y * (0.1f / 50.0f);
    data_in.mag[2] = mag.z * (0.1f / 50.0f);

    /* Aggiornamento dell'algoritmo di Sensor Fusion */
    MotionFX_propagate(mfx_state, &data_out, &data_in, &delta_time);
    MotionFX_update(mfx_state, &data_out, &data_in, &delta_time, NULL);
}

/* [DEAD-CODE:K2] DISABILITATA — verificato con grep su tutto il codebase utente
 * (escluso Debug/): zero chiamatori. Solo prototipo in .h + def qui. Il body
 * conteneva un printf con 4× %f (softfloat su F401RE, ~ms a chiamata) → flash
 * sprecato. Prototipo lasciato in mems_kinematics.h: se mai servirà, basta
 * scommentare il body. */
#if 0
void MEMS_Kinematics_printQuaternion(void)
{
  printf("Quaternion: %.3f, %.3f, %.3f, %.3f\r\n",data_out.quaternion[0] ,data_out.quaternion[1],data_out.quaternion[2],data_out.quaternion[3]);
}
#endif

/* printRotation / printHeading: chiamate da MEMS_Kinematics_process SOLO sotto
 * MEMS_KINEMATICS_VERBOSE=1 (default 0 → già di fatto morto). Tengo i body
 * perché sono il "modo debug ufficiale" del modulo; ricordati che attivare
 * il flag riempie UART2 a 100 Hz. */
void MEMS_Kinematics_printRotation(void)
{
  printf("Yaw Pitch Roll: %.3f, %.3f, %.3f\r\n",data_out.rotation[0],data_out.rotation[1],data_out.rotation[2]);
}

void MEMS_Kinematics_printHeading(void)
{
    printf("Heading: %.2f deg (err %.2f)\r\n",
           data_out.heading, data_out.headingErr);
}


/* [DEAD-CODE:K3] DISABILITATA — zero chiamatori. */
#if 0
void MEMS_Kinematics_GetQuaternion(float q[4])
{
    q[0] = data_out.quaternion[0];
    q[1] = data_out.quaternion[1];
    q[2] = data_out.quaternion[2];
    q[3] = data_out.quaternion[3];
}
#endif

void MEMS_Kinematics_GetEuler(float ypr[3])
{
    ypr[0] = data_out.rotation[0];
    ypr[1] = data_out.rotation[1];
    ypr[2] = data_out.rotation[2];
}

/* [DEAD-CODE:K3b] DISABILITATA — zero chiamatori. Yaw equivalente è già nei
 * ypr inviati via BLE stream. */
#if 0
float MEMS_Kinematics_GetHeading(void)
{
    return data_out.heading;
}
#endif

void MEMS_Kinematics_GetAccel(float acc[3])
{
    /* data_in.acc è in g, aggiornato a 100 Hz dentro MEMS_Kinematics_Update.
     * [REVIEW-FIX:8] PRIMA: se chiamata prima del primo Update, ritornava (0,0,0)
     * e arm_palm.c calcolava un palm_rel_deg fuorviante (norma quadra < EPS_ACC2
     * → ritornava 0, che però è un valore "valido" per il consumer). ORA: se
     * ancora non c'è un Update valido restituisco (0, 0, 1) — vettore di gravità
     * unitario nominale — così arm_palm vede una gravità "plausibile" e
     * pitch_arm ~ 0, evitando spike spuri nei primissimi pacchetti BLE.
     * Per ripristinare il vecchio comportamento eliminare il blocco if. */
    if (s_last_update_tick == 0U) {
        acc[0] = 0.0f;
        acc[1] = 0.0f;
        acc[2] = 1.0f;
        return;
    }
    acc[0] = data_in.acc[0];
    acc[1] = data_in.acc[1];
    acc[2] = data_in.acc[2];
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Controlla se l'interruzione è stata generata proprio dal TUO timer (es. TIM4)
    if (htim->Instance == TIM1)
    {
    	datareq = 1U; // Alza la bandierina per il while(1)
    }
}


/* [DEAD-CODE:K4] DISABILITATA — zero chiamatori. Quando servirà per la
 * cinematica del braccio (ruotare punti del modello con il quaternione
 * corrente), scommentare. */
#if 0
void MEMS_Kinematics_RotatePoint(const Point3D_t *p_in, Point3D_t *p_out)
{
    /* Estrazione del quaternione q = [x, y, z, w] */
    float qx = data_out.quaternion[0];
    float qy = data_out.quaternion[1];
    float qz = data_out.quaternion[2];
    float qw = data_out.quaternion[3];

    /* Calcolo della matrice di rotazione associata al quaternione.
       Formula ottimizzata per evitare l'uso delle classiche funzioni trigonometriche. */
    float x2 = qx + qx;
    float y2 = qy + qy;
    float z2 = qz + qz;

    float xx = qx * x2;
    float xy = qx * y2;
    float xz = qx * z2;
    float yy = qy * y2;
    float yz = qy * z2;
    float zz = qz * z2;
    float wx = qw * x2;
    float wy = qw * y2;
    float wz = qw * z2;

    float vx = p_in->x;
    float vy = p_in->y;
    float vz = p_in->z;

    /* Moltiplicazione Vettore * Matrice = Nuovo Vettore Ruotato */

    p_out->x = vx * (1.0f - (yy + zz)) + vy * (xy - wz)         + vz * (xz + wy);
    p_out->y = vx * (xy + wz)          + vy * (1.0f - (xx + zz)) + vz * (yz - wx);
    p_out->z = vx * (xz - wy)          + vy * (yz + wx)         + vz * (1.0f - (xx + yy));

}
#endif



