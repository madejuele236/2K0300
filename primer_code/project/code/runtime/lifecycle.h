#ifndef PRIMER_CODE_RUNTIME_LIFECYCLE_H_
#define PRIMER_CODE_RUNTIME_LIFECYCLE_H_

#include <cstdint>

extern int8_t run_flag;
extern uint32_t fPS;
extern int32_t it_time, encode_l_total, encode_r_total;
extern int16_t encoder_abs;

void init(void);
void Encoder_update(void);
void pid_init(void);

#endif
