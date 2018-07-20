/*
 * spxR1_tkData.c
 *
 *  Created on: 6 de dic. de 2017
 *      Author: pablo
 */

#include "spx.h"

// Este factor es porque la resistencia shunt es de 7.3 por lo que con 20mA llegamos hasta 3646 y no a 4096
#define FACTOR_CORRECCION_RSHUNT	3646

//------------------------------------------------------------------------------------
// PROTOTIPOS

// VARIABLES LOCALES
static st_data_frame pv_data_frame;

// La tarea pasa por el mismo lugar c/timerPoll secs.
#define WDG_DAT_TIMEOUT	 ( systemVars.timerPoll + 60 )

//------------------------------------------------------------------------------------
void tkData(void * pvParameters)
{

( void ) pvParameters;

uint32_t waiting_ticks;
TickType_t xLastWakeTime;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	xprintf_P( PSTR("starting tkData..\r\n\0"));

	// Configuro los INA para promediar en 128 valores.
	pub_analog_config_INAS(CONF_INAS_AVG128);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Al arrancar poleo a los 10s
    waiting_ticks = (uint32_t)(10) * 1000 / portTICK_RATE_MS;

	// loop
	for( ;; )
	{

		vTaskDelayUntil( &xLastWakeTime, waiting_ticks ); // Da el tiempo para entrar en tickless.

		// Leo analog,digital,rtc,salvo en BD e imprimo.
		pub_data_read_frame();

		// Muestro en pantalla.
		pub_data_print_frame();

		// Espero un ciclo
		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
			taskYIELD();
		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
		pub_ctl_reload_timerPoll();
		xSemaphoreGive( sem_SYSVars );

	}

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_data_read_frame(void)
{

int8_t xBytes;

	// Funcion usada para leer los datos de todos los modulos, guardarlos en memoria
	// e imprimirlos.
	// La usa por un lado tkData en forma periodica y desde el cmd line cuando se
	// da el comando read frame.

	// Leo los canales analogicos.
	// Prendo los sensores, espero un settle time de 1s, los leo y apago los sensores.
	ACH_prender_12V();
	pub_analog_config_INAS(CONF_INAS_AVG128);	// Saco a los INA del modo pwr_down
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	pub_analog_read_frame( &pv_data_frame.analog_frame);
	pub_analog_config_INAS(CONF_INAS_SLEEP);	// Pongo a los INA a dormir.
	ACH_apagar_12V();

	// Leo la bateria
	pub_analog_read_battery ( &pv_data_frame.battery );

	// Agrego el timestamp
	xBytes = RTC_read_dtime( &pv_data_frame.rtc);
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:pub_data_read_frame\r\n\0"));

}
//------------------------------------------------------------------------------------
void pub_data_print_frame(void)
{
	// Imprime el frame actual en consola

uint8_t channel;

	// HEADER
	xprintf_P(PSTR("frame: " ) );
	// timeStamp.
	xprintf_P(PSTR( "%04d%02d%02d,"),pv_data_frame.rtc.year,pv_data_frame.rtc.month,pv_data_frame.rtc.day );
	xprintf_P(PSTR("%02d%02d%02d"),pv_data_frame.rtc.hour,pv_data_frame.rtc.min, pv_data_frame.rtc.sec );

	// Valores analogicos
	// Solo muestro los que tengo configurados.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( ! strcmp ( systemVars.an_ch_name[channel], "X" ) )
			continue;

		xprintf_P(PSTR(",%s=%.02f"),systemVars.an_ch_name[channel],pv_data_frame.analog_frame.mag_val[channel] );
	}

	// bateria
	xprintf_P(PSTR(",BAT=%.02f"), pv_data_frame.battery );

	// TAIL
	xprintf_P(PSTR("\r\n\0") );

}
//------------------------------------------------------------------------------------
