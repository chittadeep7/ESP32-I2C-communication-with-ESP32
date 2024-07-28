#define MPU6050_address_W 0xD0
#define MPU6050_address_R 0xD1
#define MPU6050_address 0x68
#define who_am_i 0x75
#define pwr 0x6B
#define ACC 0x3B
#define ACK 0x00
#define NACK 0x01

void MPU6050_init(int dev_addr);

uint8_t MPU_id(int dev_addr);

void MPU6050_read(int dev_addr);

float acc(int16_t Acc_value);
float Temp(int16_t Temp_value);
float Gyro(int16_t Gyro_value);