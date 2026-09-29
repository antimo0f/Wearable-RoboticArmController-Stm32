#ifndef ARM_PALM_H_
#define ARM_PALM_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inclinazione (pitch) verticale del palmo RISPETTO al braccio, in gradi.
 *
 * Modello: per ciascun sensore prendo l'accelerometro come stima della
 * gravità nel proprio body frame (valido in quasi-statico), estraggo il
 * pitch dell'asse "forward" del chip, e sottraggo. La differenza è
 * invariante rispetto a roll/yaw del braccio: cattura solo la flesso-
 * estensione del polso nel piano sagittale.
 *
 * Convenzioni (cambia il #define in arm_palm.c se il tuo montaggio è diverso):
 *   - Asse "forward" di entrambi i chip = X.
 *
 * Output:
 *    0   → palmo allineato al braccio (polso dritto)
 *   +k   → palmo flesso in alto rispetto al braccio
 *   -k   → palmo flesso in basso rispetto al braccio
 *   Range tipico: [-180, +180], ma per ergonomia del polso ~ [-90, +90].
 *
 * @param acc_arm   Accelerometro MEMS (sul braccio), in g, body frame.
 * @param acc_palm  Accelerometro MPU6050 (sul palmo), in g, body frame.
 */
float ArmPalm_RelativePitchDeg(const float acc_arm[3], const float acc_palm[3]);

#ifdef __cplusplus
}
#endif

#endif /* ARM_PALM_H_ */
