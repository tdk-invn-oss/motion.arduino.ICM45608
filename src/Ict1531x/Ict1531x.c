/*
 *
 * Copyright (c) [2023] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */


#include "Ict1531x.h"
#include <string.h>

static void wait_us(uint32_t us)
{
	uint64_t tt1, tt2;

	tt1 = inv_ict1531x_get_time_us();
	while (1) {
		tt2 = inv_ict1531x_get_time_us();
		if (tt2 > (tt1 + us))
		break;
	}
}

void inv_ict1531x_reset_states(struct inv_ict1531x * s,
		const struct inv_ict1531x_serif * serif)
{
	memset(s, 0, sizeof(*s));
	s->serif = *serif;
}

int inv_ict1531x_poll_data(struct inv_ict1531x * s, int16_t * compass_data_lsb, int16_t * temp_data_lsb)
{
	int rc = 1;
	uint8_t data[10] = {0};

	if(s->compass_en) {
		rc = inv_ict1531x_serif_read_reg(&s->serif, ICT1531X_STATUS_REG, data, 1);
		if(rc != 0)
			return rc;

		if((data[0] & ICT1531X_STATUS_REG_DATA_READY_MASK) == 0) {
			/* not an error, but data is not ready yet */
			return 1;
		}

		/* data is ready, read relevant registers now. Reading the last data register
		 * (mag_data_z_msb) clears the data_ready bit to '0'.
		 */
		rc = inv_ict1531x_serif_read_reg(&s->serif, ICT1531X_FRAME_CNT_REG, &data[1], 9);
		if(rc != 0) {
			/* for some reason we could not read data register but DRDY has already been fired.
			 * Thus it is safer to reset compass_en flag to avoid putting driver in some
			 * undefined state.
			 */
			s->compass_en = 0;
			return rc;
		}
		
		s->frame_cnt = data[1];
		
		/* get temperature value if drdy */
		*temp_data_lsb = (((int16_t)data[3]) << 8) | data[2];

		/* get measurement values if drdy */
		compass_data_lsb[0] = (((int16_t)data[5]) << 8) | data[4];
		compass_data_lsb[1] = (((int16_t)data[7]) << 8) | data[6];
		compass_data_lsb[2] = (((int16_t)data[9]) << 8) | data[8];
	}

	return rc;
}

int inv_ict1531x_get_whoami(struct inv_ict1531x * s, uint8_t * whoami)
{
	int rc = 0;
	rc |= inv_ict1531x_serif_read_reg(&s->serif, ICT1531X_CHIP_ID_REG, whoami, 1);
	return rc;
}

int inv_ict1531x_soft_reset(struct inv_ict1531x * s)
{
	int rc = 0;
	uint8_t data;
	
	// Unlock protected register
	rc = inv_ict1531x_global_lock(s, 0);
	if (rc)
		return rc;
	
	rc = inv_ict1531x_serif_read_reg(&s->serif, ICT1531X_SEQUENCER_CTRL_REG, &data, 1);
	if (rc)
		return rc;
	data &= ~ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_MASK;
	data |= 1 << ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_POS;
	rc = inv_ict1531x_serif_write_reg(&s->serif, ICT1531X_SEQUENCER_CTRL_REG, &data, 1);
	if (rc)
		return rc;
	
	// Lock protected register
	rc = inv_ict1531x_global_lock(s, 1);
	if (rc)
		return rc;
	
	// Run MRM procedure after SW reset
	rc = inv_ict1531x_set_mrm(s);

	return rc;
}

int inv_ict1531x_set_mode(struct inv_ict1531x * s, inv_ict1531x_mode_t mode)
{
	int rc = INV_ERROR_SUCCESS;
	uint8_t data;
	
	// PULSED mode not supported
	if(mode == ICT1531X_MODE_CTRL_REG_MODE_PULSED)
		return INV_ERROR_NIMPL;
	
	// Make sure the current mode is in standby mode before switching to any other mode.
	inv_ict1531x_mode_t cur_mode;
    rc |= inv_ict1531x_get_mode(s, &cur_mode);
    if(cur_mode != ICT1531X_MODE_CTRL_REG_MODE_STANDBY) {
		uint8_t data;
		rc = inv_ict1531x_serif_read_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);
		if (rc)
			return rc;
		data &= ~ICT1531X_MODE_CTRL_REG_MODE_MASK;
		data |= ICT1531X_MODE_CTRL_REG_MODE_STANDBY;
		rc = inv_ict1531x_serif_write_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);	
		do {
			rc |= inv_ict1531x_get_mode(s, &cur_mode);
		} while((rc == 0) && (cur_mode != ICT1531X_MODE_CTRL_REG_MODE_STANDBY));
	}
	if (rc)
		return rc;
	
	// The device is already in STANDBY mode so nothing to be done.
	if (mode == ICT1531X_MODE_CTRL_REG_MODE_STANDBY)
		return rc;
		
	rc = inv_ict1531x_serif_read_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);
	if (rc)
		return rc;
	data &= ~ICT1531X_MODE_CTRL_REG_MODE_MASK;
	data |= mode;
	rc = inv_ict1531x_serif_write_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);
	
	return rc;	
}

int inv_ict1531x_get_mode(struct inv_ict1531x * s, inv_ict1531x_mode_t *cur_mode)
{
	int rc = INV_ERROR_SUCCESS;
	
	uint8_t data;
	rc = inv_ict1531x_serif_read_reg(&s->serif, ICT1531X_MODE_STATUS_REG, &data, 1);
	if (rc)
		return rc;
	*cur_mode = (inv_ict1531x_mode_t)(data & ICT1531X_MODE_CTRL_REG_MODE_MASK);
	
	return rc;
}

int inv_ict1531x_enable_sensor(struct inv_ict1531x * s, uint8_t en)
{
	int rc = 0;

	/* only support single shot mode, not PULSED mode */
	if(en){
		if(s->compass_en) {
			/* already enabled, let it complete its acquisition */
			rc = INV_ERROR;
		} else {
			rc = inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_SINGLE_SHOT);
			s->compass_en = 1;
		}
	} else {
		rc = inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_STANDBY);
		s->compass_en = 0;
	}

	return rc;
}

int inv_ict1531x_set_mrm(struct inv_ict1531x * s)
{
	int rc = 0;
	rc = inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_MRM);
	wait_us(10);
	rc |= inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_STANDBY);
	
	return rc;
}

int inv_ict1531x_global_lock(struct inv_ict1531x * s, uint8_t lock)
{
	uint8_t data;

	// Unlock the protected registers:
	// conf_calib.sys_cfg @ 0x21 and sequencer_ctrl @ 0x7C
	if(!lock)
		data = 0xCA;
	// Lock these protected registers
	else
		data = 0;

	return inv_ict1531x_serif_write_reg(&s->serif, ICT1531X_GLOBAL_LOCK_REG, &data, 1);
}
