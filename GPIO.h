/*
 * 	GPIO.h
 *	brief
 *		Prototype functions for GPIO drivers
 *  Created on: 26 aug 2023
 *  Author: L. Enrique Gortárez Ramírez, luis.gortarez@iteso.mx
 *
 */

#include <stdint.h>
#include "bits.h"
#include "fsl_clock.h"
#include "fsl_port.h"


#define SIM5 (*((volatile uint32_t *)0x40048038)) 				/* Define SIM module SCGC5 */

#define PORTCLEAR 0xFFFF
#define CLOCKA 					(0x00000200u)					/* Define CLOCKA address for SCGC5 */
#define CLOCKB 					(0x00000400u)					/* Define CLOCKB address for SCGC5 */
#define CLOCKC 					(0x00000800u)					/* Define CLOCKC address for SCGC5 */
#define CLOCKD 					(0x00001000u)					/* Define CLOCKD address for SCGC5 */
#define CLOCKE 					(0x00002000u)					/* Define CLOCKE address for SCGC5 */

#define PORT_A_BASE_ADDR       (0x40049000u)
#define PORT_B_BASE_ADDR       (0x4004A000u)
#define PORT_C_BASE_ADDR       (0x4004B000u)
#define PORT_D_BASE_ADDR       (0x4004C000u)
#define PORT_E_BASE_ADDR       (0x4004D000u)

#define GPIO_A_BASE_ADDR (((volatile uint32_t *) 0x400FF000)) 	/* Define GPIOA PDOR */
#define GPIO_B_BASE_ADDR (((volatile uint32_t *) 0x400FF040)) 	/* Define GPIOB PDOR */
#define GPIO_C_BASE_ADDR (((volatile uint32_t *) 0x400FF080)) 	/* Define GPIOC PDOR */
#define GPIO_D_BASE_ADDR (((volatile uint32_t *) 0x400FF0C0)) 	/* Define GPIOD PDOR */
#define GPIO_E_BASE_ADDR (((volatile uint32_t *) 0x400FF100)) 	/* Define GPIOE PDOR */

typedef struct
{
	  	uint32_t PCR[32];      	/* Pin Control Register */
} 		PORT_t;

#define PORT_A                   ((PORT_t*) PORT_A_BASE_ADDR)	/* Give PORT_t to PORTs */
#define PORT_B                   ((PORT_t*) PORT_B_BASE_ADDR)
#define PORT_C                   ((PORT_t*) PORT_C_BASE_ADDR)
#define PORT_D                   ((PORT_t*) PORT_D_BASE_ADDR)
#define PORT_E                   ((PORT_t*) PORT_E_BASE_ADDR)

typedef struct
{
		uint32_t PDOR;
		uint32_t PSOR; 			/* Sets selected pin in logic state 1 */
		uint32_t PCOR; 			/* Clears selected pin, 1 -> 0, 0 -> 0 */
		uint32_t PTOR; 			/* Inverts state of selected pin */
		uint32_t PDIR; 			/* Reads input of selected pin */
		uint32_t PDDR; 			/* Pin is configured as output when 1, 0 for output */
} 		GPIO_t;

#define GPIO_A 					((GPIO_t*) GPIO_A_BASE_ADDR)	/* Give GPIO_t to GPIOs */
#define GPIO_B 					((GPIO_t*) GPIO_B_BASE_ADDR)
#define GPIO_C 					((GPIO_t*) GPIO_C_BASE_ADDR)
#define GPIO_D 					((GPIO_t*) GPIO_D_BASE_ADDR)
#define GPIO_E 					((GPIO_t*) GPIO_E_BASE_ADDR)

typedef enum
{
    kPIN_Input  = 0x103U, 		/* Set pin as input */
    kPIN_Output = 0x101U,		/* Set pin as output */
} 	pin_config_t;

typedef enum
{
	GPIO_NAME_A,
	GPIO_NAME_B,
	GPIO_NAME_C,
	GPIO_NAME_D,
	GPIO_NAME_E
}	GPIO_name_t;

typedef struct
{
	uint8_t flag_port_a : 1;
	uint8_t flag_port_b : 1;
	uint8_t flag_port_c : 1;
	uint8_t flag_port_d : 1;
	uint8_t flag_port_e : 1;
} 	gpio_interrupt_flags_t;

typedef enum gpio_pin_direction
{
    kGPIO_DigitalInput,
    kGPIO_DigitalOutput
} gpio_pin_direction_t;


typedef struct _gpio_pin_config
{
    gpio_pin_direction_t pinDirection;
} gpio_pin_config_t;



void PORTA_IRQHandler(void);
void PORTC_IRQHandler(void);
void CLK_Init(uint32_t CLOCK);
void GPIO_init(void);
void PORT_Init_Pin(PORT_t* PORT, uint32_t pin, pin_config_t config);
void GPIO_Init_Pin(GPIO_t* GPIO, uint32_t pin);
void GPIO_Set_Pin(GPIO_t* GPIO, uint32_t pin);
void GPIO_Clear_Pin(GPIO_t* GPIO, uint32_t pin);
void GPIO_Invert_Pin(GPIO_t* GPIO, uint32_t pin);
uint32_t Get_GPIO_Pin(GPIO_t* GPIO, uint32_t pin);
void GPIO_clear_irq_status(GPIO_name_t GPIO);
uint8_t GPIO_get_irq_status(GPIO_name_t GPIO);
void GPIO_PinInitV2(GPIO_t* GPIO, uint32_t pin, const gpio_pin_config_t *config);
void GPIO_PortClearInterruptFlagsV2(PORT_Type *port, uint32_t mask);
