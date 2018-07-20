/*
 * spx.h
 *
 *  Created on: 20 de oct. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPXR1_H_
#define SRC_SPXR1_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <ctype.h>
#include "avr_compiler.h"
#include "clksys_driver.h"
#include <inttypes.h>

#include "TC_driver.h"
#include "pmic_driver.h"
#include "wdt_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "frtos-io.h"

#include "l_iopines.h"
#include "l_eeprom.h"
#include "l_nvm.h"
#include "l_ina3221.h"
#include "l_rtc79410.h"
#include "l_printf.h"
#include "l_ain.h"
#include "l_outputs.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "1.0.0"
#define SPX_FW_DATE "@ 20180720"

#define SPX_HW_MODELO "spxPILOTO HW:xmega256A3B R1.0"
#define SPX_FTROS_VERSION "FW:FRTOS10 TICKLESS"

// El datalogger tiene 6 canales fisicos pero 5 disponibles
// ya que uno esta para monitorear la bateria.
#define NRO_ANALOG_CHANNELS		5

#define F_CPU (32000000UL)

//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32

#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384
#define tkData_STACK_SIZE		384
#define tkOutputs_STACK_SIZE	384


#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkData_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkOutputs_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

TaskHandle_t xHandle_idle, xHandle_tkCtl,xHandle_tkCmd, xHandle_tkData, xHandle_tkOutputs;

char stdout_buff[CHAR64];

// Mensajes entre tareas
#define TK_FRAME_READY			0x01	//
#define TK_REDIAL				0x04	//

//------------------------------------------------------------------------------------
typedef enum { DEBUG_NONE = 0, DEBUG_GPRS, DEBUG_RANGEMETER, DEBUG_DIGITAL } t_debug;
typedef enum { OUT_OFF = 0, OUT_CONSIGNA, OUT_NORMAL } t_outputs;
typedef enum { CONSIGNA_DIURNA = 0, CONSIGNA_NOCTURNA } t_consigna_aplicada;
typedef enum { USER_NORMAL, USER_TECNICO } usuario_t;
//------------------------------------------------------------------------------------

// Estructura para manejar los canales ANALOGICOS
typedef struct {
	float mag_val[NRO_ANALOG_CHANNELS];
} st_analog_frame;


typedef struct {
	float analog_val[NRO_ANALOG_CHANNELS];
} st_remote_values;

// Estructura de datos manejados por la tarea DATA = ANALOGICO + DIGITAL + RANGE_METER.
typedef struct {
	RtcTimeType_t rtc;
	st_analog_frame analog_frame;
	float battery;
} st_data_frame;

#define PARAMNAME_LENGTH 6

typedef struct {
	// Variables de trabajo.

	// Configuracion de Canales analogicos
	uint16_t coef_calibracion[NRO_ANALOG_CHANNELS];
	uint8_t imin[NRO_ANALOG_CHANNELS];	// Coeficientes de conversion de I->magnitud (presion)
	uint8_t imax[NRO_ANALOG_CHANNELS];
	float mmin[NRO_ANALOG_CHANNELS];
	float mmax[NRO_ANALOG_CHANNELS];
	char an_ch_name[NRO_ANALOG_CHANNELS][PARAMNAME_LENGTH];
	char a_ch_modo[NRO_ANALOG_CHANNELS];


	uint16_t timerPoll;
	t_debug debug;

	// El checksum DEBE ser el ultimo byte del systemVars !!!!
	uint8_t checksum;

} systemVarsType;

systemVarsType systemVars;

bool startTask;
//------------------------------------------------------------------------------------
// PROTOTIPOS
//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkData(void * pvParameters);
void tkOutputs(void * pvParameters);

xSemaphoreHandle sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

// Utils
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);
void initMCU(void);
void pub_load_defaults(void);
uint8_t pub_save_params_in_NVMEE(void);
bool u_load_params_from_NVMEE(void);

// analog
void pub_analog_config_INAS( uint16_t conf_reg_value );
void pub_analog_load_defaults(void);
void pub_analog_config_channel( uint8_t channel,char *_s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax );
void pub_analog_config_timerpoll ( char *s_timerpoll );
void pub_analog_config_spanfactor ( uint8_t channel, char *s_spanfactor );
void pub_analog_read_channel ( uint8_t channel, uint16_t *raw_val, float *mag_val );
void pub_analog_read_battery ( float *mag_val );
void pub_analog_read_frame(st_analog_frame *analog_frame );
void pub_analog_prender_12vsensor ( void );
void pub_analog_apagar_12vsensor ( void );

// tkData
void pub_data_print_frame(void);
void pub_data_read_frame(void);

// tkCtl
bool pub_ctl_terminal_is_on(void);
void pub_ctl_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs );
void pub_ctl_print_wdg_timers(void);
void pub_ctl_print_stack_watermarks(void);
uint16_t pub_ctl_readTimeToNextPoll(void);
void pub_ctl_reload_timerPoll(void);

// tkOutputs
void pub_output_load_defaults(void);
void pub_output_config( char *param0, char *param1, char *param2 );
void pub_output_set_consigna_diurna(void);
void pub_output_set_consigna_nocturna(void);
void pub_output_set_outputs( char id_output, uint8_t value);

#endif /* SRC_SPXR1_H_ */
