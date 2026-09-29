#include "arm_palm.h"

#include <math.h>

#define DEG_PER_RAD  57.29577951308232f
#define EPS_ACC2     1e-12f

/* Indice del componente "forward" (0=X, 1=Y, 2=Z) per ciascun sensore.
 * Cambia in base al tuo montaggio fisico. */
#define ARM_FORWARD_IDX   0  /* MEMS sul braccio:  X forward */
#define PALM_FORWARD_IDX  0  /* MPU6050 sul palmo: X forward */

/* Segno opzionale: se nel tuo montaggio "forward up" produce un componente
 * di accelerometro positivo invece che negativo, inverti il segno. */
#define ARM_FORWARD_SIGN   (+1.0f)
#define PALM_FORWARD_SIGN  (-1.0f)


static float pitch_rad_from_accel(const float acc[3], int idx, float sign)
{
    const float n2 = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
    if (n2 < EPS_ACC2) {
        return 0.0f;
    }
    /* In statico, accelerometro = -gravity (body frame). Quando l'asse
     * "forward" punta in alto, la sua componente è negativa → negare per
     * avere "alto = positivo". */
    float s = -sign * acc[idx] / sqrtf(n2);
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    return asinf(s);
}

float ArmPalm_RelativePitchDeg(const float acc_arm[3], const float acc_palm[3])
{
    const float pitch_arm  = pitch_rad_from_accel(acc_arm,  ARM_FORWARD_IDX,  ARM_FORWARD_SIGN);
    const float pitch_palm = pitch_rad_from_accel(acc_palm, PALM_FORWARD_IDX, PALM_FORWARD_SIGN);
    return (pitch_palm - pitch_arm) * DEG_PER_RAD;
}
