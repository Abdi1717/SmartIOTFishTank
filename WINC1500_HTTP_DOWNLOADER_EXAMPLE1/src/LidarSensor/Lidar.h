#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

// Constants
#define LIDAR_ADDR (0x62 << 1)

#define LIDAR_ACQ_CMD_REG 0x00
#define LIDAR_ACQ_CMD 0x04

#define LIDAR_STATUS_REG 0x01
#define LIDAR_STATUS_BUSY_MASK 0x01
#define LIDAR_STATUS_HEALTH_MASK 0x20

#define LIDAR_DELTA_VELOCITY_REG 0x09
#define LIDAR_DIST_REG 0x8f

// LiDAR status struct
typedef struct {
	char busy;
	char healthy;
} lidar_status_t;

// Function prototypes
void lidar_read_status(lidar_status_t *lidar_status);
void lidar_die(void);
void lidar_write_acq_cmd(void);
uint16_t lidar_read_distance(void);
int8_t lidar_read_delta_velocity(void);

#endif // LIDAR_H
