Simple MPU6050 driver for ESP32 that you can use in your project to read accelerometer, temperature and gyroscope values.

=>The MPU_read() function reads all the data inside the buffer.
    =>inside the task function conversion is done and data is printed out.
    =>main contain sthe I2C configuration and task creation. 

****Tested on esp32 and wokwi using esp-idf v5.2.2 *****

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── mpu6050.c
|   |__ mpu6060.h
└── README.md               
```

