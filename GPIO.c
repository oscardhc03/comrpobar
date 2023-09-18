/*
 * 	GPIO.c
 *	brief
 *		Here the functions have been implemented for initiating GPIOS in pins and getting inputs
 *  Created on: 11 sep 2023
 *  Author: L. Enrique Gortárez Ramírez, luis.gortarez@iteso.mx
 *  		O. Daniel Sánchez Corona   , ie719971@iteso.mx
 *
 */

#include "GPIO.h"

volatile static gpio_interrupt_flags_t g_intr_status_flag = {0};

void PORTA_IRQHandler(void)
{
	g_intr_status_flag.flag_port_a = true;
	GPIO_PortClearInterruptFlagsV2(PORTA, PORTCLEAR);
}

void PORTC_IRQHandler(void)
{
	g_intr_status_flag.flag_port_c = true;
	GPIO_PortClearInterruptFlagsV2(PORTC, PORTCLEAR);
}

void CLK_Init(uint32_t CLOCK)
{
	CLOCK_SetSimSafeDivs(); 	/* ALWAYS HAVE THIS */
	SIM |= CLOCK;
}

void GPIO_init()
{


	CLK_Init(CLOCKA);
	CLK_Init(CLOCKC);

	gpio_pin_config_t gpio_input_config = {
				        kGPIO_DigitalInput
				    };


	const port_pin_config_t button_config = {
				kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
				kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
				kPORT_PassiveFilterEnable,                               /* Passive filter is disabled */
				kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
				kPORT_HighDriveStrength,                                 /* High drive strength is configured */
				kPORT_MuxAsGpio,                                         /* Pin is configured as PTA4 */
				kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
			  };


	GPIO_PinInitV2(GPIO_A, 4u, &gpio_input_config);
	PORT_SetPinConfig(PORTA, 4u, &button_config);
	PORT_SetPinInterruptConfig(PORTA, 4u, kPORT_InterruptFallingEdge);

	GPIO_PinInitV2(GPIO_C, 6u, &gpio_input_config);
	PORT_SetPinConfig(PORTC, 6u, &button_config);
	PORT_SetPinInterruptConfig(PORTC, 6u, kPORT_InterruptFallingEdge);

}

void PORT_Init_Pin(PORT_t* PORT, uint32_t pin, pin_config_t config)
{
	PORT->PCR[pin] = config;
}

void GPIO_Init_Pin(GPIO_t* GPIO, uint32_t pin)
{
	GPIO->PDDR |= (bit_1 << pin);
}

void GPIO_Set_Pin(GPIO_t* GPIO, uint32_t pin)
{
	GPIO->PSOR = pin;
}

void GPIO_Clear_Pin(GPIO_t* GPIO, uint32_t pin)
{
	GPIO->PCOR = pin;
}

void GPIO_Invert_Pin(GPIO_t* GPIO, uint32_t pin)
{
	GPIO->PTOR = bit_1 << pin;
}

uint32_t Get_GPIO_Pin(GPIO_t* GPIO, uint32_t pin)
{
	uint32_t input = 0u;
	input = GPIO->PDIR;
	input = input & (bit_1 << pin);
	return input;
}

void GPIO_clear_irq_status(GPIO_name_t GPIO)
{
	switch(GPIO)
	{
		case 0:
			g_intr_status_flag.flag_port_a = false;
			break;
		case 1:
			g_intr_status_flag.flag_port_b = false;
			break;
		case 2:
			g_intr_status_flag.flag_port_c = false;
			break;
		case 3:
			g_intr_status_flag.flag_port_d = false;
			break;
		case 4:
			g_intr_status_flag.flag_port_e = false;
			break;
		default:
			g_intr_status_flag.flag_port_a = false;
	}
}

uint8_t GPIO_get_irq_status(GPIO_name_t GPIO)
{
	uint8_t status = 0;

	switch(GPIO)
	{
	case 0:
		status = g_intr_status_flag.flag_port_a;
		break;
	case 1:
		status = g_intr_status_flag.flag_port_b;
		break;
	case 2:
		status = g_intr_status_flag.flag_port_c;
		break;
	case 3:
		status = g_intr_status_flag.flag_port_d;
		break;
	case 4:
		status = g_intr_status_flag.flag_port_e;
		break;
	default:
		status = g_intr_status_flag.flag_port_a;
	}

	return(status);
}

void GPIO_PinInitV2(GPIO_t* GPIO, uint32_t pin, const gpio_pin_config_t *config)
{
    assert(NULL != config);

    if (config->pinDirection == kGPIO_DigitalInput)
    {
        // Set the pin as an input
        GPIO->PDDR &= ~(1UL << pin);
    }
    else if (config->pinDirection == kGPIO_DigitalOutput)
    {
        // Set the pin as an output
        GPIO->PDDR |= (1UL << pin);
    }

}

void GPIO_PortClearInterruptFlagsV2(PORT_Type *port, uint32_t mask){
	assert(NULL != port);
	port->ISFR = mask;
}
