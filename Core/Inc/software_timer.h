#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_
#include <stdint.h>

extern volatile int32_t timer2_count;
extern volatile int32_t timer2_flag;

void SetupTimer2(int32_t duration);
void timer2Run(void);

#endif /* INC_SOFTWARE_TIMER_H_ */
