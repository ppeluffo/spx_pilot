/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 */

#include "spx.h"

//------------------------------------------------------------------------------------
static void pv_tkCtl_init_system(void);
static void pv_tkCtl_wink_led(void);
static void pv_tkCtl_ajust_timerPoll(void);

static uint16_t time_to_next_poll;

// Timpo que espera la tkControl entre round-a-robin
#define TKCTL_DELAY_S	5

//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	pv_tkCtl_init_system();

	xprintf_P( PSTR("\r\nstarting tkControl..\r\n\0"));

	for( ;; )
	{
		// Para entrar en tickless.
		// Cada 5s hago un chequeo de todo. En particular esto determina el tiempo
		// entre que activo el switch de la terminal y que esta efectivamente responde.
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

		pv_tkCtl_wink_led();
		pv_tkCtl_ajust_timerPoll();
		// Cada ciclo reseteo el wdg para que no expire.
		WDT_Reset();

	}
}
//------------------------------------------------------------------------------------
static void pv_tkCtl_init_system(void)
{

	// Al comienzo leo este handle para asi usarlo para leer el estado de los stacks.
	// En la medida que no estoy usando la taskIdle podria deshabilitarla. !!!
	xHandle_idle = xTaskGetIdleTaskHandle();

	// Leo los parametros del la EE y si tengo error, cargo por defecto
	if ( ! u_load_params_from_NVMEE() ) {
		pub_load_defaults();
		xprintf_P( PSTR("\r\nLoading defaults !!\r\n\0"));
	}

	time_to_next_poll = systemVars.timerPoll;

	// Arranco el RTC. Si hay un problema lo inicializo.
	RTC_start();

	// Habilito a arrancar al resto de las tareas
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_tkCtl_wink_led(void)
{
	// Prendo los leds
	IO_set_LED_KA();
	vTaskDelay( ( TickType_t)( 5 / portTICK_RATE_MS ) );
	// Apago
	IO_clr_LED_KA();

}
//------------------------------------------------------------------------------------
static void pv_tkCtl_ajust_timerPoll(void)
{
	if ( time_to_next_poll > TKCTL_DELAY_S )
		time_to_next_poll -= TKCTL_DELAY_S;
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
uint16_t pub_ctl_readTimeToNextPoll(void)
{
	return(time_to_next_poll);
}
//------------------------------------------------------------------------------------
void pub_ctl_reload_timerPoll(void)
{
	time_to_next_poll = systemVars.timerPoll;
}
//------------------------------------------------------------------------------------
