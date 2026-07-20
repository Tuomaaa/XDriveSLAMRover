#ifndef HC_SR04_H
#define HC_SR04_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define HC_SR04_INVALID_MM UINT16_MAX

typedef struct {
  uint16_t right_mm;
  uint16_t back_mm;
  uint8_t status;
} HC_SR04_Result;

void HC_SR04_Init(void);
void HC_SR04_StartMeasurement(void);
uint8_t HC_SR04_IsComplete(void);
HC_SR04_Result HC_SR04_GetResult(void);

#ifdef __cplusplus
}
#endif

#endif /* HC_SR04_H */