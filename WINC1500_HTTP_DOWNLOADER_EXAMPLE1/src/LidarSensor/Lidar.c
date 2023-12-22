
/*
 * Lidar.c
 *
 * Created: 12/18/2023 1:24:39 AM
 *  Author: AbdiMPC
 */ 


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "I2cDriver\I2cDriver.h"



// Constants
const int LIDAR_ADDR = 0x62 << 1;

const char LIDAR_ACQ_CMD_REG = 0x00;
const char LIDAR_ACQ_CMD = 0x04;

const char LIDAR_STATUS_REG = 0x01;
const char LIDAR_STATUS_BUSY_MASK = 0x01;
const char LIDAR_STATUS_HEALTH_MASK = 0x20;

const char LIDAR_DELTA_VELOCITY_REG = 0x09;
const char LIDAR_DIST_REG = 0x8f;

// Error handling function
void die(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vprintf(fmt, args);
	printf("\r\n");
	va_end(args);
	exit(1);
}

// LiDAR status struct
typedef struct {
	char busy;
	char healthy;
} lidar_status_t;

// Function to read LiDAR sensor's status


void lidar_read_status(lidar_status_t *lidar_status) {
    I2C_Data i2cData;
    char status;
    i2cData.address = LIDAR_ADDR;
    i2cData.msgOut = &LIDAR_STATUS_REG;
    i2cData.lenOut = 1;
    i2cData.msgIn = &status;
    i2cData.lenIn = 1;

    // Poll the status register until the measurement is complete
    do {
        if (I2cWriteDataWait(&i2cData, portMAX_DELAY) != ERROR_NONE ||
            I2cReadDataWait(&i2cData, 0, portMAX_DELAY) != ERROR_NONE) {
            die("lidar: read_status: failed to read status register");
        }
    } while (status & LIDAR_STATUS_BUSY_MASK);

    lidar_status->busy = status & LIDAR_STATUS_BUSY_MASK;
    lidar_status->healthy = (status & LIDAR_STATUS_HEALTH_MASK) >> 5;
}




// Function to handle LiDAR error state
void lidar_die() {
	lidar_status_t lidar_status;
	lidar_read_status(&lidar_status);
	
	if (!lidar_status.healthy) {
		die("lidar: sensor is not healthy");
	}
}


void lidar_write_acq_cmd() {
	I2C_Data i2cData;
	char buf[2] = {LIDAR_ACQ_CMD_REG, LIDAR_ACQ_CMD};

	i2cData.address = LIDAR_ADDR;
	i2cData.msgOut = buf;
	i2cData.lenOut = 2;

	if (I2cWriteDataWait(&i2cData, portMAX_DELAY) != ERROR_NONE) {
		die("lidar: write_acq_cmd: failed to write acquire command");
	}
}


uint16_t lidar_read_distance() {
	I2C_Data i2cData;
	char reg = LIDAR_DIST_REG;
	char buf[2];

	i2cData.address = LIDAR_ADDR;
	i2cData.msgOut = &reg;
	i2cData.lenOut = 1;
	i2cData.msgIn = buf;
	i2cData.lenIn = 2;

	if (I2cWriteDataWait(&i2cData, portMAX_DELAY) != ERROR_NONE) {
		die("lidar: read_distance: failed to select distance registers for read");
	}
	if (I2cReadDataWait(&i2cData, 0, portMAX_DELAY) != ERROR_NONE) {
		die("lidar: read_distance: failed to read distance registers");
	}

	return (uint16_t)(buf[1] | (buf[0] << 8));
}


int8_t lidar_read_delta_velocity() {
	I2C_Data i2cData;
	char buf;
	char reg = LIDAR_DELTA_VELOCITY_REG;

	i2cData.address = LIDAR_ADDR;
	i2cData.msgOut = &reg;
	i2cData.lenOut = 1;
	i2cData.msgIn = &buf;
	i2cData.lenIn = 1;

	if (I2cWriteDataWait(&i2cData, portMAX_DELAY) != ERROR_NONE) {
		die("lidar: read_delta_velocity: failed to select delta velocity register for read");
	}
	if (I2cReadDataWait(&i2cData, 0, portMAX_DELAY) != ERROR_NONE) {
		die("lidar: read_delta_velocity: failed to read delta velocity register");
	}

	return (int8_t)buf;
}
