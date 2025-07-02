/*
 * register_map.c
 * FC_AHRS/FC_IMU/BMI323/BMI323.c
 *
 *  Created on: June 19, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_IMU_BMI323_REGISTER_MAP_H_
#define INC_FC_AHRS_FC_IMU_BMI323_REGISTER_MAP_H_


// USER BANK 0 REGISTER
#define CHIP_ID (0x00)
#define ERR_REG (0x01)
#define STATUS (0x02)
#define ACC_DATA_X (0x03)
#define ACC_DATA_Y (0x04)
#define ACC_DATA_Z (0x05)
#define GYR_DATA_X (0x06)
#define GYR_DATA_Y (0x07)
#define GYR_DATA_Z (0x08)
#define TEMP_DATA (0x09)
#define SENSOR_TIME_O (0x0A)
#define SENSOR_TIME_1 (0x0B)
#define SAT_FLAGS (0x0C)
#define INT_STATUS_INT1 (0x0D)
#define INT_STATUS_INT2 (0x0E)
#define INT_STATUS_IBI (0xOF)
#define FEATURE_I00 (0x10)
#define FEATURE_I01 (0x11)
#define FEATURE_I02 (0x12)
#define FEATURE_I03 (0x13)
#define FEATURE_IO_STATUS (0x14)
#define FIFO_FILL_LEVEL (0x15)
#define FIFO_DATA (0x16)
#define ACC_CONF (0x20)
#define GYR_CONF (0x21)
#define ALT_ACC_CONF (0x28)
#define ALT_GYR_CONF (0x29)
#define ALT_CONF (0x2A)
#define ALT_STATUS (0x2B)
#define FIFO_WATERMARK (0x35)
#define FIFO_CONF (0x36)
#define FIFO_CTRL (0x37)
#define IO_INT_CTRL (0x38)
#define INT_CONF (0x39)
#define INT_MAP1 (0x3A)
#define INT_MAP2 (0x3B)
#define FEATURE_CTRL (0x40)
#define FEATURE_DATA_ADDR (0x41)
#define FEATURE_DATA_TX (0x42)
#define FEATURE_DATA_STATUS (0x43)
#define FEATURE_ENGINE_STATUS (0x45)
#define FEATURE_EVENT_EXT (0x47)
#define IO_PDN_CTRL (0x4F)
#define IO_SPI_IF (0x50)
#define IO_PAD_STRENGTH (0x51)
#define IO_I2C_IF (0x52)
#define IO_ODR_DEVIATION (0x53)
#define ACC_DP_OFF_X (0x60)
#define ACC_DP_DGAIN_X (0x61)
#define ACC_DP_OFF_Y (0x62)
#define ACC_DP_DGAIN_Y (0x63)
#define ACC_DP_OFF_Z (0x64)
#define ACC_DP_DGAIN_Z (0x65)
#define GYR_DP_OFF_X (0x66)
#define GYR_DP_DGAIN_X (0x67)
#define GYR_DP_OFF_Y (0x68)
#define GYR_DP_DGAIN_Y (0x69)
#define GYR_DP_OFF_Z (0x6A)
#define GYR_DP_DGAIN_Z (0x6B)
#define I3C_TC_SYNC_TPH (0x70)
#define I3C_TC_SYNC_TU (0x71)
#define I3C_TC_SYNC_ODR (0x72)
#define CMD (0x7E)
#define CFG_RES (0x7F)


#endif /* INC_FC_AHRS_FC_IMU_BMI323_REGISTER_MAP_H_ */
