/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "FRTOS-CMD.h"
#include "spx.h"

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

static void pv_cmd_INA(uint8_t cmd_mode );
static void pv_cmd_sens12V(void);
static void pv_cmd_rwNVMEE(uint8_t cmd_mode );
static void pv_cmd_rwRTC(uint8_t cmd_mode );
static void pv_cmd_wrOUT8814(void);
static void pv_cmd_rwACH(uint8_t cmd_mode );

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);

#define WR_CMD 0
#define RD_CMD 1

char aux_buffer[32];
RtcTimeType_t rtc;

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );
	FRTOS_CMD_register( "status\0", cmdStatusFunction );
	FRTOS_CMD_register( "config\0", cmdConfigFunction );
	FRTOS_CMD_register( "kill\0", cmdKillFunction );

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdUSB,ioctl_SET_TIMEOUT, &ticks );

	xprintf_P( PSTR("starting tkCmd..\r\n\0") );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		//while ( CMD_read( (char *)&c, 1 ) == 1 ) {
		while ( frtos_read( fdUSB, (char *)&c, 1 ) == 1 ) {
			FRTOS_CMD_process(c);
		}
	}
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

uint8_t channel;

	xprintf_P( PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );

	// SIGNATURE ID
	memset(aux_buffer,'\0', sizeof(aux_buffer));
	NVMEE_readID((char *)&aux_buffer);
	xprintf_P( PSTR("uID=%s\r\n\0"), aux_buffer);

	// Fecha y Hora
	pv_cmd_rwRTC( RD_CMD );

	xprintf_P( PSTR("  timerPoll: [%d s]/%d\r\n\0"),systemVars.timerPoll, pub_ctl_readTimeToNextPoll() );

	// Configuracion de canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( systemVars.a_ch_modo[channel] == 'R') {
			xprintf_P( PSTR("  a%d(*) [%d-%d mA/ %.02f,%.02f | %04d | %s]\r\n\0"),channel, systemVars.imin[channel],systemVars.imax[channel],systemVars.mmin[channel],systemVars.mmax[channel], systemVars.coef_calibracion[channel], systemVars.an_ch_name[channel] );
		} else {
			xprintf_P( PSTR("  a%d( ) [%d-%d mA/ %.02f,%.02f | %04d | %s]\r\n\0"),channel, systemVars.imin[channel],systemVars.imax[channel],systemVars.mmin[channel],systemVars.mmax[channel], systemVars.coef_calibracion[channel], systemVars.an_ch_name[channel] );
		}
	}

	// Valores actuales:
	pub_data_print_frame();

}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	cmdClearScreen();

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// RTC
	// write rtc YYMMDDhhmm
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(WR_CMD);
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0")))  {
		pv_cmd_INA(WR_CMD);
		return;
	}

	// ANALOG
	// write analog {ina_id} conf128
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0"))) {
		pv_cmd_rwACH(WR_CMD);
		return;
	}

	// SENS12V
	// write sens12V {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("SENS12V\0"))) {
		pv_cmd_sens12V();
		return;
	}

	// NVMEE
	// write nvmee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0"))) {
		pv_cmd_rwNVMEE(WR_CMD);
		return;
	}

	// OUT 8814
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	//       out pulse (A/B) (+/-) (ms)
	//       out power {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0"))) {
		pv_cmd_wrOUT8814();
		return;
	}

	// OUTPUTS
	// write outputs {x,x}
	// Debo esperar para que se carguen los condensadores
/*	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0"))) {
		pub_output_set_outputs( 'A', atoi(argv[2]) );
		vTaskDelay( ( TickType_t)( 3000 / portTICK_RATE_MS ) );
		pub_output_set_outputs( 'B', atoi(argv[3]) );
		pv_snprintfP_OK();
		return;
	}
*/
	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

uint16_t raw_val;
float mag_val;

	FRTOS_CMD_makeArgv();

	// RTC
	// read rtc
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(RD_CMD);
		return;
	}

	// FRAME
	// read frame
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) ) {
		pub_data_read_frame();
		pub_data_print_frame();
		return;
	}

	// INA
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))) {
		pv_cmd_INA(RD_CMD);
		return;
	}

	// NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0"))) {
		pv_cmd_rwNVMEE(RD_CMD);
		return;
	}

	// ARAW
	// read araw {ch, bat}
	if (!strcmp_P( strupr(argv[1]), PSTR("ARAW\0"))) {
		pv_cmd_rwACH(RD_CMD);
		return;
	}


	// ACH { 0..4}
	// read ach x
	if (!strcmp_P( strupr(argv[1]), PSTR("ACH\0"))) {
		if ( atoi(argv[2]) > 4) {
			pv_snprintfP_ERR();
			return;
		}
		pub_analog_read_channel( atoi(argv[2]),&raw_val, &mag_val );
		xprintf_P( PSTR("CH[%02d] raw=%d,mag=%.02f\r\n\0"),atoi(argv[2]),raw_val, mag_val );
		return;
	}


	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	xprintf_P( PSTR("\x1B[2J\0"));
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{


	FRTOS_CMD_makeArgv();


	// config outputs
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0")) ) {
//		pub_output_config( argv[2], argv[3], argv[4] );
		pv_snprintfP_OK();
		return;
	}

	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		pub_save_params_in_NVMEE();
		pv_snprintfP_OK();
		return;
	}

	// config analog {0..2} aname imin imax mmin mmax
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) ) {
		pub_analog_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7] );
		pv_snprintfP_OK();
		return;
	}

	// config timerpoll
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0")) ) {
		pub_analog_config_timerpoll( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		pub_load_defaults();
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE\0"))) {
		xprintf_P( PSTR("-write\r\n\0"));
		xprintf_P( PSTR("  rtc YYMMDDhhmm\r\n\0"));
		xprintf_P( PSTR("  consigna (diurna|nocturna)\r\n\0"));
		xprintf_P( PSTR("  nvmee {pos} {string}\r\n\0"));
		xprintf_P( PSTR("  ina (id) conf {value}, sens12V {on|off}\r\n\0"));
		xprintf_P( PSTR("  analog {ina_id} conf128 \r\n\0"));
		xprintf_P( PSTR("  out { (enable|disable),(set|reset),(sleep|awake),(ph01|ph10) } {A/B}\r\n\0"));
		xprintf_P( PSTR("      valve (open|close) (A|B) (ms)\r\n\0"));
		xprintf_P( PSTR("      power {on|off}\r\n\0"));
		return;
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		xprintf_P( PSTR("-read\r\n\0"));
		xprintf_P( PSTR("  rtc, frame\r\n\0"));
		xprintf_P( PSTR("  nvmee {pos} {lenght}\r\n\0"));
		xprintf_P( PSTR("  ina {id} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
		xprintf_P( PSTR("  ach {0..4}, battery\r\n\0"));
		xprintf_P( PSTR("  araw {ch, bat} \r\n\0"));
		return;
	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		xprintf_P( PSTR("-reset\r\n\0"));
	return;

	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		xprintf_P( PSTR("-config\r\n\0"));
		xprintf_P( PSTR("  analog {0..4} aname imin imax mmin mmax\r\n\0"));
		xprintf_P( PSTR("  outputs {off}|{normal o0 o1}|{consigna hhmm_dia hhmm_noche}\r\n\0"));
		xprintf_P( PSTR("  timerpoll\r\n\0"));
		xprintf_P( PSTR("  default\r\n\0"));
		xprintf_P( PSTR("  save\r\n\0"));
		return;

	} else {

		// HELP GENERAL
		xprintf_P( PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		xprintf_P( PSTR("Available commands are:\r\n\0"));
		xprintf_P( PSTR("-cls\r\n\0"));
		xprintf_P( PSTR("-help\r\n\0"));
		xprintf_P( PSTR("-status\r\n\0"));
		xprintf_P( PSTR("-reset...\r\n\0"));
		xprintf_P( PSTR("-write...\r\n\0"));
		xprintf_P( PSTR("-read...\r\n\0"));
		xprintf_P( PSTR("-config...\r\n\0"));

	}

	xprintf_P( PSTR("\r\n\0"));

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{

	FRTOS_CMD_makeArgv();

	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf_P( PSTR("ok\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf_P( PSTR("error\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_cmd_INA(uint8_t cmd_mode )
{

uint16_t val;
uint8_t ina_id;
char data[3];
int8_t xBytes;

	// write ina id conf {value}
	if ( cmd_mode == WR_CMD ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0")) ) {
			ina_id = atoi(argv[2]);
			val = atoi( argv[4]);
			data[0] = ( val & 0xFF00 ) >> 8;
			data[1] = ( val & 0x00FF );
			xBytes = INA_write( ina_id, INA3231_CONF, data, 2 );
			if ( xBytes == -1 )
				xprintf_P(PSTR("ERROR: I2C:INA:pv_cmd_INA\r\n\0"));

			pv_snprintfP_OK();
			return;
		}
	}

	// read ina id {conf|chxshv|chxbusv|mfid|dieid}
	if ( cmd_mode == RD_CMD ) {

		ina_id = atoi(argv[2]);

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0"))) {
			xBytes = INA_read(  ina_id, INA3231_CONF, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH1_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH1_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH2_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH2_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH3_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH3_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("MFID\0"))) {
			xBytes = INA_read(  ina_id, INA3221_MFID, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("DIEID\0"))) {
			xBytes = INA_read(  ina_id, INA3221_DIEID, data, 2 );
		} else {
			pv_snprintfP_ERR();
			return;
		}

		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:INA:pv_cmd_INA\r\n\0"));

		val = ( data[0]<< 8 ) + data	[1];
		xprintf_P( PSTR("INAID=%d\r\n\0"), ina_id);
		xprintf_P( PSTR("VAL=0x%04x\r\n\0"), val);
		pv_snprintfP_OK();
		return;

	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_sens12V(void)
{
	// sens12V on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	xprintf_P( PSTR("cmd ERROR: ( write sens12V on{off} )\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
static void pv_cmd_rwNVMEE(uint8_t cmd_mode )
{
	// Hace prueba de lectura y escritura de la memoria internan EE del micro
	// que es la que usamos para guardar la configuracion.

int xBytes = 0;
uint8_t length = 0;
char buffer[32];
char *p;

	// read nvmee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {

		xBytes = NVMEE_read( (uint16_t)(atoi(argv[2])), buffer, (uint8_t)(atoi(argv[3]) ) );
		if ( xBytes > 0 ) {
			xprintf_P( PSTR( "%s\r\n\0"),buffer);
		}
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write nvmee pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		xBytes = NVMEE_write( (uint16_t)(atoi(argv[2])), argv[3], length );
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC(uint8_t cmd_mode )
{

char datetime[24];
RtcTimeType_t rtc;
bool retS;
int8_t xBytes;

	if ( cmd_mode == WR_CMD ) {
		RTC_str2rtc(argv[2], &rtc);				// Convierto el string YYMMDDHHMM a RTC.
		xBytes = RTC_write_dtime(&rtc);		// Grabo el RTC
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC\r\n\0"));

		( xBytes > 0)? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if ( cmd_mode == RD_CMD ) {
		xBytes = RTC_read_dtime(&rtc);
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC\r\n\0"));

		RTC_rtc2str(datetime,&rtc);
		xprintf_P( PSTR("%s\r\n\0"), datetime );
		return;
	}

}
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
static void pv_cmd_wrOUT8814(void)
{
	// write out { (enable|disable),(set|reset),(sleep|awake),(ph01|ph10) } {A/B}
	//             power {on|off}
	//             valve (open|close) (A|B) (ms)

	// write out enable (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) ) {
		( OUT_enable_pin( toupper(argv[3][0]), 1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out disable (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("DISABLE\0")) ) {
		( OUT_enable_pin( toupper(argv[3][0]), 0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out set
	if (!strcmp_P( strupr(argv[2]), PSTR("SET\0")) ) {
		( OUT_reset_pin (1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out reset
	if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
		( OUT_reset_pin (0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out sleep
	if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
		( OUT_sleep_pin (1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out awake
	if (!strcmp_P( strupr(argv[2]), PSTR("AWAKE\0")) ) {
		( OUT_sleep_pin (0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out ph01 (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("PH01\0")) ) {
		( OUT_phase_pin( toupper(argv[3][0]), 1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out ph10 (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("PH10\0")) ) {
		( OUT_phase_pin( toupper(argv[3][0]), 0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out power on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("POWER\0")) ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
			OUT_power_on();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
			OUT_power_off();
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	//  write out valve (open|close) (A|B) (ms)
	if (!strcmp_P( strupr(argv[2]), PSTR("VALVE\0")) ) {

		// Proporciono corriente.
		OUT_power_on();
		// Espero 10s que se carguen los condensasores
		vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

		if (!strcmp_P( strupr(argv[3]), PSTR("OPEN\0")) ) {
			( OUT_valve( toupper(argv[4][0]), V_OPEN, atoi(argv[5]) )  > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
			OUT_power_off();
			return;
		}
		if (!strcmp_P( strupr(argv[3]), PSTR("CLOSE\0")) ) {
			( OUT_valve( toupper(argv[4][0]), V_CLOSE, atoi(argv[5]) )  > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
			OUT_power_off();
			return;
		}

		OUT_power_off();
		pv_snprintfP_ERR();
		return;
	}

	pv_snprintfP_ERR();
	return;

}

//------------------------------------------------------------------------------------
static void pv_cmd_rwACH(uint8_t cmd_mode )
{

uint16_t val = 0;
uint8_t channel;

	// read aCh {ch}, bat
	if ( cmd_mode == RD_CMD ) {
		// Bateria
		if (!strcmp_P( strupr( (char *)argv[2]), PSTR("BAT\0"))) {
			val = ACH_read_battery();
			xprintf_P( PSTR("BAT=%d\r\n\0"), val);
			pv_snprintfP_OK();
			return;
		}

		// Canales
		channel = atoi(argv[2]);
		if (  channel > 4 ) {
			pv_snprintfP_ERR();
			return;
		} else {
			val = ACH_read_channel( channel );
			xprintf_P( PSTR("CH%0d=%d\r\n\0"), channel, val);
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// write ach {id} conf128
	if ( cmd_mode == WR_CMD ) {
		ACH_config_avg128(atoi(argv[2]));
		pv_snprintfP_OK();
		return;
	}
}
//------------------------------------------------------------------------------------
