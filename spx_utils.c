/*
 * xmega01_utils.c
 *
 *  Created on: 1 de nov. de 2016
 *      Author: pablo
 */

#include "spx.h"

#define RTC32_ToscBusy()        !( VBAT.STATUS & VBAT_XOSCRDY_bm )
void RTC32_ToscEnable( bool use1khz );

//-----------------------------------------------------------
void initMCU(void)
{
	// Inicializa los pines del micro

	// LEDS:
	IO_config_LED_KA();
	IO_config_LED_COMMS();

	// ANALOG: SENSOR VCC CONTROL
	IO_config_SENS_12V_CTL();

	// TERMINAL CTL PIN
	IO_config_TERMCTL_PIN();

	// OUT8814
	OUT_config();

	// TICK:
	//IO_config_TICK();

	// PWR_SLEEP
	IO_config_PWR_SLEEP();
//	IO_set_PWR_SLEEP();

}
//-----------------------------------------------------------
void u_configure_systemMainClock(void)
{
/*	Configura el clock principal del sistema
	Inicialmente se arranca en 2Mhz.
	La configuracion del reloj tiene 2 componentes: el clock y el oscilador.
	OSCILADOR:
	Primero debo elejir cual oscilador voy a usar para alimentar los prescalers que me den
	el clock del sistema y esperar a que este este estable.
	CLOCK:
	Elijo cual oscilador ( puedo tener mas de uno prendido ) va a ser la fuente principal
	del closck del sistema.
	Luego configuro el prescaler para derivar los clocks de los perifericos.
	Puedo por ultimo 'lockear' esta configuracion para que no se cambie por error.
	Los registros para configurar el clock son 'protegidos' por lo que los cambio
	utilizando la funcion CCPwrite.

	Para nuestra aplicacion vamos a usar un clock de 32Mhz.
	Como vamos a usar el ADC debemos prestar atencion al clock de perifericos clk_per ya que luego
	el ADC clock derivado del clk_per debe estar entre 100khz y 1.4Mhz ( AVR1300 ).

	Opcionalmente podriamos deshabilitar el oscilador de 2Mhz para ahorrar energia.
*/

#if SYSMAINCLK == 32
	// Habilito el oscilador de 32Mhz
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 32 Mhz ).
	//
#endif

#if SYSMAINCLK == 8
	// Habilito el oscilador de 32Mhz y lo divido por 4
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// Pongo el prescaler A por 4 y el B y C en 0.
	CLKSYS_Prescalers_Config( CLK_PSADIV_4_gc, CLK_PSBCDIV_1_1_gc );

	//
#endif

#if SYSMAINCLK == 2
	// Este es el oscilador por defecto por lo cual no tendria porque configurarlo.
	// Habilito el oscilador de 2Mhz
	OSC.CTRL |= OSC_RC2MEN_bm;
	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC2MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 2Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC2M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 2 Mhz ).
	//
#endif

//#ifdef configUSE_TICKLESS_IDLE
	// Para el modo TICKLESS
	// Configuro el RTC con el osc externo de 32Khz
	// Pongo como fuente el xtal externo de 32768 contando a 32Khz.
	//CLK.RTCCTRL = CLK_RTCSRC_TOSC32_gc | CLK_RTCEN_bm;
	//do {} while ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) );

	// Disable RTC interrupt.
	// RTC.INTCTRL = 0x00;
	//
	// Si uso el RTC32, habilito el oscilador para 1ms.

	RTC32_ToscEnable(true);
//#endif

	// Lockeo la configuracion.
	CCPWrite( &CLK.LOCK, CLK_LOCK_bm );

}
//-----------------------------------------------------------
void RTC32_ToscEnable( bool use1khz )
{
	/* Enable 32 kHz XTAL oscillator, with 1 kHz or 1 Hz output. */
	if (use1khz)
		VBAT.CTRL |= ( VBAT_XOSCEN_bm | VBAT_XOSCSEL_bm );
	else
		VBAT.CTRL |= ( VBAT_XOSCEN_bm );

	RTC32.PER = 10;
	RTC32.CNT = 0;

	/* Wait for oscillator to stabilize before returning. */
//	do { } while ( RTC32_ToscBusy() );
}
//-----------------------------------------------------------
void pub_load_defaults(void)
{

	pub_analog_load_defaults();

}
//-----------------------------------------------------------
void u_configure_RTC32(void)
{
	// El RTC32 lo utilizo para desperarme en el modo tickless.
	// V-bat needs to be reset, and activated
	VBAT.CTRL |= VBAT_ACCEN_bm;
	// Este registro esta protegido de escritura con CCP.
	CCPWrite(&VBAT.CTRL, VBAT_RESET_bm);

	// Pongo el reloj en 1.024Khz.
	VBAT.CTRL |=  VBAT_XOSCSEL_bm | VBAT_XOSCFDEN_bm ;

	// wait for 200us see AVR1321 Application note page 8
	_delay_us(200);

	// Turn on 32.768kHz crystal oscillator
	VBAT.CTRL |= VBAT_XOSCEN_bm;

	// Wait for stable oscillator
	while(!(VBAT.STATUS & VBAT_XOSCRDY_bm));

	// Disable RTC32 module before setting counter values
	RTC32.CTRL = 0;

	// Wait for sync
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );

	// EL RTC corre a 1024 hz y quiero generar un tick de 10ms,
	RTC32.PER = 1024;
	RTC32.CNT = 0;

	// Interrupt: on Overflow
	RTC32.INTCTRL = RTC32_OVFINTLVL_LO_gc;

	// Enable RTC32 module
	RTC32.CTRL = RTC32_ENABLE_bm;

	/* Wait for sync */
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );
}
//-----------------------------------------------------------
uint8_t pub_save_params_in_NVMEE(void)
{
	// Calculo el checksum del systemVars.
	// Considero el systemVars como un array de chars.

uint8_t *p;
uint8_t checksum;
uint16_t data_length;
uint16_t i;

	// Calculo el checksum del systemVars.
	systemVars.checksum = 0;
	data_length = sizeof(systemVars);
	p = (uint8_t *)&systemVars;
	checksum = 0;
	// Recorro todo el systemVars considerando c/byte como un char, hasta
	// llegar al ultimo ( checksum ) que no lo incluyo !!!.
	for ( i = 0; i < (data_length - 1) ; i++) {
		checksum += p[i];
	}
	checksum = ~checksum;
	systemVars.checksum = checksum;

	// Guardo systemVars en la EE
	NVMEE_write_buffer(0x00, &systemVars, sizeof(systemVars));

	return(checksum);

}
//------------------------------------------------------------------------------------
bool u_load_params_from_NVMEE(void)
{
	// Leo el systemVars desde la EE.
	// Calculo el checksum. Si no coincide es que hubo algun
	// error por lo que cargo el default.


uint8_t *p;
uint8_t stored_checksum, checksum;
uint16_t data_length;
uint16_t i;

	// Leo de la EE es systemVars.
	NVMEE_read_buffer(0x00, (char *)&systemVars, sizeof(systemVars));

	// Guardo el checksum que lei.
	stored_checksum = systemVars.checksum;

	// Calculo el checksum del systemVars leido
	systemVars.checksum = 0;
	data_length = sizeof(systemVars);
	p = ( uint8_t *)&systemVars;	// Recorro el systemVars como si fuese un array de int8.
	checksum = 0;
	for ( i = 0; i < ( data_length - 1 ); i++) {
		checksum += p[i];
	}
	checksum = ~checksum;

	if ( stored_checksum != checksum ) {
		return(false);
	}

	return(true);
}
//------------------------------------------------------------------------------------
