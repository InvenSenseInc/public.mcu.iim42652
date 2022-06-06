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

/** @defgroup DriverIxm42xxxTransport Ixm42xxx driver transport
 *  @brief    Low-level Ixm42xxx register access
 *  @ingroup  DriverIxm42xxx
 *  @{
 */

/** @file Ixm42xxxTransport.h
 * Low-level Ixm42xxx register access
 */

#ifndef _INV_IXM42XXX_TRANSPORT_H_
#define _INV_IXM42XXX_TRANSPORT_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* forward declaration */
struct inv_ixm42xxx;


/** @brief enumeration  of serial interfaces available on ixm42xxx */
typedef enum
{
  IXM42XXX_UI_I2C,
  IXM42XXX_UI_SPI4,
  IXM42XXX_UI_I3C,
  IXM42XXX_AUX1_SPI3,
  IXM42XXX_AUX2_SPI3
  
} IXM42XXX_SERIAL_IF_TYPE_t;
 
/** @brief basesensor serial interface
 */
struct inv_ixm42xxx_serif {
	void *     context;
	int      (*read_reg)(struct inv_ixm42xxx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len);
	int      (*write_reg)(struct inv_ixm42xxx_serif * serif, uint8_t reg, const uint8_t * buf, uint32_t len);
	int      (*configure)(struct inv_ixm42xxx_serif * serif);
	uint32_t   max_read;
	uint32_t   max_write;
	IXM42XXX_SERIAL_IF_TYPE_t serif_type;
};

/** @brief transport interface
 */
struct inv_ixm42xxx_transport {
	struct inv_ixm42xxx_serif serif; /**< Warning : this field MUST be the first one of struct inv_ixm42xxx_transport */

	/** @brief Contains mirrored values of some IP registers */
	struct register_cache {
		uint8_t intf_cfg_1_reg;   /**< INTF_CONFIG1, Bank: 0, Address: 0x4D */
		uint8_t pwr_mngt_0_reg;   /**< PWR_MGMT_0, Bank: 0, Address: 0x4E */
		uint8_t gyro_cfg_0_reg;   /**< GYRO_CONFIG0, Bank: 0, Address: 0x4F */
		uint8_t accel_cfg_0_reg;  /**< ACCEL_CONFIG0, Bank: 0, Address: 0x50 */
		uint8_t tmst_cfg_reg;     /**< TMST_CONFIG, Bank: 0, Address: 0x54 */
		uint8_t bank_sel_reg;     /**< MPUREG_REG_BANK_SEL, All banks, Address 0x76*/
	} register_cache; /**< Store mostly used register values on SRAM. 
	                    *  MPUREG_OTP_SEC_STATUS_B1 and MPUREG_INT_STATUS registers
	                    *  are read before the cache has a chance to be initialized. 
	                    *  Therefore, these registers shall never be added to the cache 
						*  Registers from bank 1,2,3 or 4 shall never be added to the cache
	                    */
};

/** @brief Init cache variable.
 * @return            0 in case of success, -1 for any error
 */
int inv_ixm42xxx_init_transport(struct inv_ixm42xxx * s);

/** @brief Reads data from a register on Ixm42xxx.
 * @param[in] reg    register address to be read
 * @param[in] len    number of byte to be read
 * @param[out] buf   output data from the register
 * @return            0 in case of success, -1 for any error
 */
int inv_ixm42xxx_read_reg(struct inv_ixm42xxx * s, uint8_t reg, uint32_t len, uint8_t * buf);

/** @brief Writes data to a register on Ixm42xxx.
 * @param[in] reg    register address to be written
 * @param[in] len    number of byte to be written
 * @param[in] buf    input data to write
 * @return            0 in case of success, -1 for any error
 */
int inv_ixm42xxx_write_reg(struct inv_ixm42xxx * s, uint8_t reg, uint32_t len, const uint8_t * buf);

/** @brief Enable MCLK so that MREG are clocked and system beyond SOI can be safely accessed
 * @param[out] idle_en Value of IDLE bit prior to function access, this might be useful to understand if MCLK was already enabled or not
 * @return            0 in case of success, -1 for any error
 */

#ifdef __cplusplus
}
#endif

#endif /* _INV_IXM42XXX_TRANSPORT_H_ */

/** @} */
