#ifndef LAB2_MAIN_H
#define LAB2_MAIN_H
 
void blink_led(GPIO_TypeDef * port, uint16_t pins);
 
void turn_on_led(GPIO_TypeDef * port, uint16_t pins);
 
void clock_init(void);

void set_clock(void);

void gpio_init(void);

void tim_init(void);
#endif /* LAB2_MAIN_H */
