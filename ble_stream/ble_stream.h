#ifndef BLE_STREAM_H_
#define BLE_STREAM_H_

#include <stdint.h>
#include "hand_acc_GY521.h"

#ifdef __cplusplus
extern "C" {
#endif

void BLE_Stream_Init(void);

void BLE_Stream_SetMPU6050(const MPU6050_data_out *d);

void BLE_Stream_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_STREAM_H_ */
