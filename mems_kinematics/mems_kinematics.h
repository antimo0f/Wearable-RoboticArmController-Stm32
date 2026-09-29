/**
  ******************************************************************************
  * @file    mems_kinematics.h
  * @brief   Libreria per calcolo orientamento tramite MotionFX e rotazione di punti 3D.
  *          Progettata per essere esportabile in altri progetti STM32.
  ******************************************************************************
  */

#ifndef MEMS_KINEMATICS_H
#define MEMS_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Struttura che rappresenta un punto o un vettore nello spazio 3D.
 */
typedef struct {
    float x;
    float y;
    float z;
} Point3D_t;

void MEMS_Kinematics_Init(void);
void MEMS_Kinematics_process(void);
void MEMS_Kinematics_Update(void);
void MEMS_Kinematics_RotatePoint(const Point3D_t *p_in, Point3D_t *p_out);
void MEMS_Kinematics_printQuaternion(void);
void MEMS_Kinematics_printRotation(void);
void MEMS_Kinematics_printHeading(void);

void MEMS_Kinematics_GetQuaternion(float q[4]);
void MEMS_Kinematics_GetEuler(float ypr[3]);
float MEMS_Kinematics_GetHeading(void);
void MEMS_Kinematics_GetAccel(float acc[3]);  /* g, body frame */

#ifdef __cplusplus
}
#endif

#endif /* MEMS_KINEMATICS_H */
