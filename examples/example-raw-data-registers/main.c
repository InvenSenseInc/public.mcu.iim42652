/*
 * __________________________________________________________________
 *
 * Copyright (C) [2022] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY  AND FITNESS. IN NO EVENT SHALL
 * THE AUTHOR BE LIABLE  FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * __________________________________________________________________
 */

#include "example-raw-data-registers.h"

/* InvenSense utils */
#include "Message.h"

/* std */
#include <stdio.h>


/* --------------------------------------------------------------------------------------
 *  Example configuration
 * -------------------------------------------------------------------------------------- */

/*
 * Select UART port on which INV_MSG() will be printed.
 */
#define LOG_UART_ID INV_UART_SENSOR_CTRL

/* 
 * Set of timers used throughout standalone applications 
 */
#define TIMEBASE_TIMER INV_TIMER1
#define DELAY_TIMER    INV_TIMER2

/* 
 * Select communication link between SmartMotion and IXM42xxx 
 */
#define SERIF_TYPE IXM42XXX_UI_SPI4
//#define SERIF_TYPE IXM42XXX_UI_I2C

/* 
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

/*
 * Set power mode flag
 * Set this flag to run example in low-noise mode.
 * Reset this flag to run example in low-power mode.
 * Note : low-noise mode is not available with sensor data frequencies less than 12.5Hz.
 */
#define IS_LOW_NOISE_MODE 1

/* 
 * Set this to 0 if you want to test timestamping mechanism without CLKIN 32k capability.
 * Please set a hardware bridge between PA17 (from MCU) and CLKIN pins (to ICM).
 * Warning: This option is not available for all IXM42XXX. Please check the datasheet.
 */
#define USE_CLK_IN 0


/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */

/* 
 * Buffer to keep track of the timestamp when ixm42xxx data ready interrupt fires.
 * The buffer can contain up to 64 items in order to store one timestamp for each packet in FIFO.
 */
//RINGBUFFER(timestamp_buffer, 64, uint64_t);


/* --------------------------------------------------------------------------------------
 *  Static variables
 * -------------------------------------------------------------------------------------- */

/* Flag set from ixm42xxx device irq handler */
static volatile int irq_from_device;

#define TO_MASK(a) (1U << (unsigned)(a))

enum gpio_inv_pin_num {
	INV_GPIO_INT1 = 0,         /* Connected to the INT1 pin of the Invensense chip. */
	INV_GPIO_INT2,             /* Connected to the INT2 pin of the Invensense chip. */
	INV_GPIO_FSYNC,            /* Connected to the FSYNC pin of the Invensense chip. */
	INV_GPIO_3RD_PARTY_INT1,   /* Connected to the interrupt pin of the 3rd party chip. */
	INV_GPIO_SW0_BUTTON,       /* Connected to the SW0 button. */
	INV_GPIO_CLKIN,            /* Output SLCK on PA17 pin. Set a hardawre bridge to connect to CLKIN pin of the Invensense chip. */
	INV_GPIO_MAX
};

/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static void SetupMCUHardware(struct inv_ixm42xxx_serif * icm_serif);
static void ext_interrupt_cb(void * context, unsigned int int_num);
static void check_rc(int rc, const char * msg_context);
void msg_printer(int level, const char * str, va_list ap);

int inv_io_hal_read_reg(struct inv_ixm42xxx_serif * serif, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	//switch (serif->serif_type) {
	//	case IXM42XXX_UI_SPI4:
	//		return inv_spi_master_read_register(INV_SPI_AP, reg, rlen, rbuffer);
	//	case IXM42XXX_UI_I2C:
	//		while(inv_i2c_master_read_register(ICM_I2C_ADDR, reg, rlen, rbuffer)) {
	//			inv_delay_us(32000); // Loop in case of I2C timeout
	//		}
	//		return 0;
	//	default:
	//		return -1;
	//}
	return 0;
}

int inv_io_hal_write_reg(struct inv_ixm42xxx_serif * serif, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	//int rc;

	//switch (serif->serif_type) {
	//	case IXM42XXX_UI_SPI4:
	//		for(uint32_t i=0; i<wlen; i++) {
	//			rc = inv_spi_master_write_register(INV_SPI_AP, reg+i, 1, &wbuffer[i]);
	//			if(rc)
	//				return rc;
	//		}
	//		return 0;
	//	case IXM42XXX_UI_I2C:
	//		while(inv_i2c_master_write_register(ICM_I2C_ADDR, reg, wlen, wbuffer)) {
	//			inv_delay_us(32000); // Loop in case of I2C timeout
	//		}
	//		return 0;
	//	default:
	//		return -1;
	//}
	return 0;
}

/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

int main(void)
{
	int rc = 0;
	struct inv_ixm42xxx_serif ixm42xxx_serif;
	
	/* Initialize MCU hardware */
	SetupMCUHardware(&ixm42xxx_serif);
	
	/* Initialize Ixm42xxx */
	rc = SetupInvDevice(&ixm42xxx_serif);
	check_rc(rc, "error while setting up INV device");

	/* Configure Ixm42xxx */
	/* /!\ In this example, the data output frequency will be the faster  between Accel and Gyro odr */
	rc = ConfigureInvDevice((uint8_t )IS_LOW_NOISE_MODE,
	                                  IXM42XXX_ACCEL_CONFIG0_FS_SEL_4g,
	                                  IXM42XXX_GYRO_CONFIG0_FS_SEL_2000dps,
	                                  IXM42XXX_ACCEL_CONFIG0_ODR_1_KHZ,
	                                  IXM42XXX_GYRO_CONFIG0_ODR_1_KHZ,
	                        (uint8_t )USE_CLK_IN);
	
	check_rc(rc, "error while configuring INV device");
	
	do {
		/* Poll device for data */		
		if (irq_from_device & TO_MASK(INV_GPIO_INT1)) {
			rc = GetDataFromInvDevice();
			check_rc(rc, "error while processing FIFO");

			//inv_disable_irq();
			irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
			//inv_enable_irq();
		}
		
	} while(1);
}



/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group and GPIO so that MCU can receive interrupts from IXM42xxx
 *   - a microsecond timer requested by Ixm42xxx driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a serial link to communicate from MCU to Ixm42xxx
 */
static void SetupMCUHardware(struct inv_ixm42xxx_serif * icm_serif)
{
	//inv_board_hal_init();
	
	/* configure UART */
	//config_uart(LOG_UART_ID);

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	INV_MSG(INV_MSG_LEVEL_INFO, "##################################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Example Raw data registers   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "##################################");

	/*
	 * Configure input capture mode GPIO connected to pin EXT3-9 (pin PB03).
	 * This pin is connected to Ixm42xxx INT1 output and thus will receive interrupts 
	 * enabled on INT1 from the device.
	 * A callback function is also passed that will be executed each time an interrupt
	 * fires.
	*/
	//inv_gpio_sensor_irq_init(INV_GPIO_INT1, ext_interrupt_cb, 0);

	/* Init timer peripheral for delay */
	//inv_delay_init(DELAY_TIMER);

	/*
	 * Configure the timer for the timebase
	 */
	//inv_timer_configure_timebase(1000000);
	//inv_timer_enable(TIMEBASE_TIMER);

//#if USE_CLK_IN
//	rtc_timer_init(NULL);
//	/* Output 32kHz SLCK to PA17, it is up to user to connect it or not at board level to have CLKIN capability */
//	inv_gpio_output_clk_on_pin(INV_GPIO_CLKIN);
//#endif

	/* Initialize serial inteface between MCU and Ixm42xxx */
	icm_serif->context   = 0;        /* no need */
	icm_serif->read_reg  = inv_io_hal_read_reg;
	icm_serif->write_reg = inv_io_hal_write_reg;
	icm_serif->max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;
	//inv_io_hal_init(icm_serif);
}


/*
 * Ixm42xxx interrupt handler.
 * Function is executed when an Ixm42xxx interrupt rises on MCU.
 * This function get a timestamp and store it in the timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * are implemented for shared variable timestamp_buffer.
 */
static void ext_interrupt_cb(void * context, unsigned int int_num)
{
	(void)context;

#if USE_CLK_IN
	/* 
	 * Read timestamp from the RTC derived from SLCK, clocking CLKIN
	 */
	//uint64_t timestamp = rtc_timer_get_time_us();
#else
	/* 
	 * Read timestamp from the timer dedicated to timestamping 
	 */
	//uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);
#endif

	//if(int_num == INV_GPIO_INT1) {
	//	if (!RINGBUFFER_FULL(&timestamp_buffer))
	//		RINGBUFFER_PUSH(&timestamp_buffer, &timestamp);
	//}

	irq_from_device |= TO_MASK(int_num);
}


/*
 * Helper function to check RC value and block programm exectution
 */
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}


/*
 * Printer function for message facility
 */
void msg_printer(int level, const char * str, va_list ap)
{
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * s[INV_MSG_LEVEL_MAX] = {
	    "",    // INV_MSG_LEVEL_OFF
	    "[E] ", // INV_MSG_LEVEL_ERROR
	    "[W] ", // INV_MSG_LEVEL_WARNING
	    "[I] ", // INV_MSG_LEVEL_INFO
	    "[V] ", // INV_MSG_LEVEL_VERBOSE
	    "[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

	//inv_uart_mngr_puts(LOG_UART_ID, out_str, idx);
}


/* --------------------------------------------------------------------------------------
 *  Extern functions definition
 * -------------------------------------------------------------------------------------- */

/*
 * Ixm42xxx driver needs to get time in us. Let's give its implementation here.
 */
uint64_t inv_ixm42xxx_get_time_us(void)
{
//#if USE_CLK_IN
//	return rtc_timer_get_time_us();
//#else
//	return inv_timer_get_counter(TIMEBASE_TIMER);
//#endif
	return 0;
}

/*
 * Clock calibration module needs to disable IRQ. Thus inv_helper_disable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_disable_irq(void)
{
	//inv_disable_irq();
}

/*
 * Clock calibration module needs to enable IRQ. Thus inv_helper_enable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_enable_irq(void)
{
	//inv_enable_irq();
}

/*
 * Ixm42xxx driver needs a sleep feature from external device. Thus inv_ixm42xxx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 */
void inv_ixm42xxx_sleep_us(uint32_t us)
{
	//inv_delay_us(us);
}
