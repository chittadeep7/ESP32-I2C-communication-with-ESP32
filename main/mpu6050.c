#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "mpu6050.h"

uint8_t buf[14];
int16_t AX, AY, AZ, TEMP, GX, GY, GZ;

void MPU6050_init(int dev_addr)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, pwr, true);
  i2c_master_write_byte(cmd, 0x01, true);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(2000));
  i2c_cmd_link_delete(cmd);
}

//0x68 should be returned (slave address of MPU6050)
uint8_t MPU_id(int dev_addr)             
{
  uint8_t data;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, who_am_i, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &data, 0x1);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(2000));
  i2c_cmd_link_delete(cmd);
  return data;

}

//Reads the data from 0x3B in a buffer
void MPU6050_read(int dev_addr)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, ACC, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
  for(uint8_t i = 0; i < 13; i++){
    i2c_master_read_byte(cmd, &buf[i], ACK);
  }
  i2c_master_read_byte(cmd, &buf[13], NACK);
 
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(2000));
  i2c_cmd_link_delete(cmd);
  AX = ((buf[0] << 8) | buf[1]);
  AY = ((buf[2] << 8) | buf[3]);
  AZ = ((buf[4] << 8) | buf[5]);
  TEMP = ((buf[6] << 8) | buf[7]);
  GX = ((buf[8] << 8) | buf[9]);
  GY = ((buf[10] << 8) | buf[11]);
  GZ = ((buf[12] << 8) | buf[13]);
  
}

float acc(int16_t Acc_value)            //accelerometer raw value / acc sensitivity factor
{
  return Acc_value / 16384.0;

}
float Temp(int16_t Temp_value)          //Conversion to celcius
{
  return (Temp_value / 340.0) + 36.53;

}
float Gyro(int16_t Gyro_value)          //gyroscope conversion
{
  return Gyro_value / 131.0;

}
//task to print out all the data
void task(void *parameter)
{
  MPU6050_init(MPU6050_address);
  uint8_t id = MPU_id(MPU6050_address);

  if(id == 0x68){
    printf("MPU6050 connected\n");
  }
  else
    printf("MPU6050 not found\n");
  while(1)
  {
    MPU6050_read(MPU6050_address);
    printf("Ax: %0.2f\t", acc(AX));
    printf("Ay: %0.2f\t", acc(AY));
    printf("Az: %0.2f\t", acc(AZ));
    printf("Temp: %0.2f\t\r", Temp(TEMP));
    printf("Gx: %0.2f\t", Gyro(GX));
    printf("Gy: %0.2f\t", Gyro(GY));
    printf("Gz: %0.2f\t\r\n", Gyro(GZ));
    
    vTaskDelay(pdMS_TO_TICKS(2000));

  }
}

void app_main(void)
{   i2c_config_t conf = {                        //I2C configuration
      .mode = I2C_MODE_MASTER,
      .sda_io_num = 26,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = 27,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000,            //100kHz
      .clk_flags = 0
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  printf("I2C configured\n");

  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  printf("I2C driver installed\n");
  
  xTaskCreate(task, "task 1", 2048, NULL, 2, NULL);
}