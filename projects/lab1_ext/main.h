#ifndef LAB1_MAIN_H
#define LAB1_MAIN_H

void clock_init(void);

/*
* int8_t clock - type of source clock for PLL. 1 - HSE, 0 - HSI
*/
void set_clock(int8_t clock);

void switch_clock(void);

#endif //LAB1_MAIN_H
