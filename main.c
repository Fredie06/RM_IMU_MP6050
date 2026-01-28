/*
 * Copyright (c) 2026 Fredie06 ryb1424276506@163.com  
 * SPDX-License-Identifier: MIT
 * IMU attitude estimate on RT-Thread
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_spi.h"
#include "math.h"
#define GYRO_PIN    GET_PIN(B, 0)
#define ACCEL_PIN   GET_PIN(A, 4)
#define DEG_TO_RAD (3.14159265358979323846f/180.0f)
#define RAD_TO_DEG  57.29577951308232f
#define X -0.000136433f
#define Y -0.002502767f
#define Z -0.001086646f
/*一些关于姿态解算算法所需的变量的定义*/


float gx;
float gy;
float gz;
float ax;
float ay;
float az;
float q0=1;
float q1=0;
float q2=0;
float q3=0;
float gyro_roll=0.0f;
float gyro_pitch=0.0f;
float gyro_yaw=0.0f;

float accel_roll=0.0f;
float accel_pitch=0.0f;

/*设备名称和线程名称*/
rt_device_t SPI_BUS         = RT_NULL;
rt_device_t UART1           = RT_NULL;
rt_base_t   gyro_cs_pin     = GYRO_PIN;
rt_base_t   accel_cs_pin    = ACCEL_PIN;
rt_device_t gyro            = RT_NULL;
rt_device_t accel           = RT_NULL;
rt_thread_t mahony_ahrs_IMU = RT_NULL;
rt_thread_t Data_Send       = RT_NULL;
rt_thread_t sensor_data_get = RT_NULL;
/*一些用于数据传输的全局变量*/
uint8_t send_data[4]={0x00,0x00,0x80,0x7F};
float temprature;
static rt_bool_t gyro_data_enable = RT_FALSE;   // 陀螺仪数据
static rt_bool_t accel_data_enable = RT_FALSE;  // 加速度计数据
static rt_bool_t euler_data_enable = RT_FALSE;  // 欧拉角数据
static rt_bool_t temp_data_enable = RT_FALSE;  // 温度计数据
struct justfloat_data_accel datapack_accel;
struct justfloat_data_gyro  datapack_gyro;
struct justfloat_data_Euler Eular_Angle;
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
/*对结构体变量进行的定义*/
static struct rt_spi_device spi_dev_gyro;
static struct rt_spi_device spi_dev_accel;
static struct ahrs_sensor sensor_inst;
static struct attitude atti_inst;
struct ahrs_sensor *sensor = &sensor_inst;
struct attitude *atti = &atti_inst;
/*用于VOFA的数据传输的相关结构体*/
struct justfloat_data_gyro
{
    float gyro[3];
    uint8_t tail[4];
};
struct justfloat_data_accel
{
    float accel[3];
    uint8_t tail[4];
};
struct justfloat_data_Euler
{
    float Euler[3];
    uint8_t tail[4];
};
struct ahrs_sensor
{
    float wx;
    float wy;
    float wz;
    float ax;
    float ay;
    float az;
};
struct attitude
{
    float roll;
    float pitch;
    float yaw;
};
struct justfloat_temp
{
    float temp[1];
    uint8_t tail[4];
};


/*所有自定义函数*/
void acc(void)
{
     gyro_data_enable = RT_FALSE;
     accel_data_enable = RT_TRUE;   /* 修正：开启加速度数据发送 */
     euler_data_enable = RT_FALSE;
     temp_data_enable = RT_FALSE;
}
MSH_CMD_EXPORT(acc, Enable Accel Data);

void gyro_data(void)
{
     gyro_data_enable = RT_TRUE;
     accel_data_enable = RT_FALSE;
     euler_data_enable = RT_FALSE;
     temp_data_enable = RT_FALSE;
}
MSH_CMD_EXPORT(gyro_data,Enable Gyro Data);

void angle(void)
{
     gyro_data_enable = RT_FALSE;
     accel_data_enable = RT_FALSE;
     euler_data_enable = RT_TRUE;
     temp_data_enable = RT_FALSE;
}
MSH_CMD_EXPORT(angle,Enable Eular Data);
void temp(void)
{
    gyro_data_enable = RT_FALSE;
    accel_data_enable = RT_FALSE;
    euler_data_enable = RT_FALSE;
    temp_data_enable = RT_TRUE;
}
MSH_CMD_EXPORT(temp, Enable Temp Data);

void mahony_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  /* 增益与积分项（可调整） */
  const float Ki = 0.5f;    /* 如需积分，可设大于0 */
  const float Kp = 1.5f;
  /* 积分项需跨次调用保存 */
  static float integralFBx = 0.0f;
  static float integralFBy = 0.0f;
  static float integralFBz = 0.0f;
  /* 采样周期：请根据实际读取频率设置（例如 0.05 = 20Hz）*/
  const float dt = 1.0f / 200.0f;

  /* 把陀螺度/s -> 弧度/s */
  gx = sensor->wx * DEG_TO_RAD;
  gy = sensor->wy * DEG_TO_RAD;
  gz = sensor->wz * DEG_TO_RAD;
  ax = sensor->ax;
  ay = sensor->ay;
  az = sensor->az;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = 1/sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (Ki > 0.0f)
    {
      integralFBx += Ki * halfex * dt;
      integralFBy += Ki * halfey * dt;
      integralFBz += Ki * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += 2*Kp * halfex;
    gy += 2*Kp * halfey;
    gz += 2*Kp * halfez;
  }

  // Integrate rate of change of quaternion
  /* 将角速度用于四元数微分（注意已为 rad/s），预乘 dt/2 */
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1/sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  atti->roll = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll     -pi----pi
  atti->pitch = asinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                // pitch    -pi/2----pi/2
  atti->yaw = atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  // yaw      -pi----pi

 
}


void imu_temperature_get(float *temperature)
{
    uint8_t temp[2];
    uint8_t reg = 0x22 | 0x80;
    rt_pin_write(ACCEL_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    rt_spi_send_then_recv(&spi_dev_accel, &reg, 1, temp, 2);
    rt_pin_write(ACCEL_PIN, PIN_HIGH);
    rt_thread_mdelay(1);
    uint16_t temp_unsigned = (temp[1] * 8) + (temp[0] / 32);
    int16_t temp_signed;
    if(temp_unsigned > 1023)
    {
        temp_signed = temp_unsigned - 2048;
    }
    else
    {
        temp_signed = temp_unsigned;
    }
    *temperature = 23.0f + (float)temp_signed * 0.125f;
}

void imu_gyro_write(uint8_t reg, uint8_t data)
{
    rt_pin_write(GYRO_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    rt_spi_send_then_send(&spi_dev_gyro, &reg, 1, &data, 1);

    rt_pin_write(GYRO_PIN, PIN_HIGH);
    rt_thread_mdelay(1);
}

void imu_accel_write(uint8_t reg, uint8_t data)
{
    rt_pin_write(ACCEL_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    rt_spi_send_then_send(&spi_dev_accel, &reg, 1, &data, 1);

    rt_pin_write(ACCEL_PIN, PIN_HIGH);
    rt_thread_mdelay(1);
}
void imu_gyro_get(float *gyro_data)
{
    uint8_t data_buf[6];
    rt_pin_write(GYRO_PIN, PIN_LOW);
    rt_thread_mdelay(1);
    struct rt_spi_message msg1, msg2;

    uint8_t GYRO_READ_CMD[1] = {0x02 | 0x80};//set read bit

    msg1.send_buf = GYRO_READ_CMD;
    msg1.recv_buf = RT_NULL;
    msg1.length = 1;
    msg1.cs_take = 1;
    msg1.cs_release = 0;
    msg1.next = &msg2;

    msg2.send_buf = RT_NULL;
    msg2.recv_buf = data_buf;
    msg2.length = 6;
    msg2.cs_take = 0;
    msg2.cs_release = 1;
    msg2.next = RT_NULL;

    rt_spi_transfer_message(&spi_dev_gyro, &msg1);

    uint16_t x = (data_buf[1] << 8) | data_buf[0];
    uint16_t y = (data_buf[3] << 8) | data_buf[2];
    uint16_t z = (data_buf[5] << 8) | data_buf[4];

    gyro_data[0] = (float)(int16_t)x / 16.384f +X; // Convert to dps
    gyro_data[1] = (float)(int16_t)y / 16.384f +Y;
    gyro_data[2] = (float)(int16_t)z / 16.384f +Z;

    rt_pin_write(GYRO_PIN, PIN_HIGH);
}

void imu_accel_get(float *accel_data)
{
    uint8_t data_buf[7];

    rt_pin_write(ACCEL_PIN, PIN_LOW);
    rt_thread_mdelay(1);
    struct rt_spi_message msg1, msg2;

    uint8_t ACCEL_READ_CMD[1] = {0x12 | 0x80};//set read bit

    msg1.send_buf = ACCEL_READ_CMD;
    msg1.recv_buf = RT_NULL;
    msg1.length = 1;
    msg1.cs_take = 1;
    msg1.cs_release = 0;
    msg1.next = &msg2;

    msg2.send_buf = RT_NULL;
    msg2.recv_buf = data_buf;
    msg2.length = 7;
    msg2.cs_take = 0;
    msg2.cs_release = 1;
    msg2.next = RT_NULL;

    rt_spi_transfer_message(&spi_dev_accel, &msg1);

    uint16_t x = (data_buf[2] << 8) | data_buf[1];
    uint16_t y = (data_buf[4] << 8) | data_buf[3];
    uint16_t z = (data_buf[6] << 8) | data_buf[5];

    accel_data[0] = (float)(int16_t)x * 0.00179443359375f; 
    accel_data[1] = (float)(int16_t)y * 0.00179443359375f;
    accel_data[2] = (float)(int16_t)z * 0.00179443359375f;

    rt_pin_write(ACCEL_PIN, PIN_HIGH);
}


/*所有线程的入口函数*/
static void mahony_ahrs_updateIMU_thread_entry(void)
{
    while(1)
    {
        mahony_ahrs_updateIMU(sensor,atti);
        Eular_Angle.Euler[0]=atti->pitch;
        Eular_Angle.Euler[1]=atti->yaw;
        Eular_Angle.Euler[2]=-atti->roll;
        Eular_Angle.tail[0] = 0x00;
        Eular_Angle.tail[1] = 0x00;
        Eular_Angle.tail[2] = 0x80;
        Eular_Angle.tail[3] = 0x7F;
        rt_thread_mdelay(5);
    }
}


static void sensor_data_get_entry(void)
{
    while(1)
    {
        datapack_accel.tail[0] = 0x00;
        datapack_accel.tail[1] = 0x00;
        datapack_accel.tail[2] = 0x80;
        datapack_accel.tail[3] = 0x7F;
        imu_accel_get(datapack_accel.accel);
        sensor->ax=datapack_accel.accel[0];
        sensor->ay=datapack_accel.accel[1];
        sensor->az=datapack_accel.accel[2];

        datapack_gyro.tail[0] = 0x00;
        datapack_gyro.tail[1] = 0x00;
        datapack_gyro.tail[2] = 0x80;
        datapack_gyro.tail[3] = 0x7F;
        imu_gyro_get(datapack_gyro.gyro);
        sensor->wx=datapack_gyro.gyro[0];
        sensor->wy=datapack_gyro.gyro[1];
        sensor->wz=datapack_gyro.gyro[2];
        imu_temperature_get(&temprature);

        rt_thread_mdelay(50);
    }
}

static void data_send_thread_entry(void)
{
    while(1)
    {
        if(gyro_data_enable==RT_TRUE && accel_data_enable==RT_FALSE && euler_data_enable==RT_FALSE && temp_data_enable==RT_FALSE)
        {
            if (UART1 != RT_NULL)
            {
                rt_device_write(UART1, 0, &datapack_gyro, sizeof(datapack_gyro));
                rt_thread_mdelay(50);
            }
        }
        
        else if(gyro_data_enable==RT_FALSE && accel_data_enable==RT_TRUE && euler_data_enable==RT_FALSE && temp_data_enable==RT_FALSE)
        {
            if (UART1 != RT_NULL)
            {
                rt_device_write(UART1, 0, &datapack_accel, sizeof(datapack_accel));
                rt_thread_mdelay(50);
            }
        }
        
        else if(gyro_data_enable==RT_FALSE && accel_data_enable==RT_FALSE && euler_data_enable==RT_TRUE && temp_data_enable==RT_FALSE)
        {
            if (UART1 != RT_NULL)
            {
                rt_device_write(UART1, 0, &Eular_Angle, sizeof(Eular_Angle));
                rt_thread_mdelay(50);
            }
        }
        
        else if(gyro_data_enable==RT_FALSE && accel_data_enable==RT_FALSE && euler_data_enable==RT_FALSE && temp_data_enable==RT_TRUE)
        {
             if (UART1 != RT_NULL)
             {
                 rt_device_write(UART1, 0, &temprature, sizeof(temprature));
                 rt_device_write(UART1, 0, (uint8_t *)send_data, sizeof(send_data));
                 rt_thread_mdelay(50);
             }
        }
        else
        {
            if (UART1 != RT_NULL)
             {
                rt_kprintf("WAITING!");
                rt_thread_mdelay(500);
             }
        }
    }
}

/*main函数*/
int main(void)
{
    UART1=rt_device_find("uart1");
    if(UART1==RT_NULL)
    {
        rt_kprintf("UART FIND FAILED\n");
    }
    else
    {
        rt_kprintf("UART FIND SUCCESS\n");
        config.bufsz=128;
        rt_device_control(UART1, RT_DEVICE_CTRL_CONFIG, &config);
        rt_err_t open_ret = rt_device_open(UART1, RT_DEVICE_OFLAG_RDWR);
        rt_kprintf("UART1 open ret = %d\n", open_ret);
    }
    struct rt_spi_configuration cfg_gyro;
    cfg_gyro.data_width=8;
    cfg_gyro.max_hz=5250000;
    cfg_gyro.mode=RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;


    struct rt_spi_configuration cfg_accel;
    cfg_accel.data_width=8;
    cfg_accel.max_hz=5250000;
    cfg_accel.mode=RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    

    rt_err_t result_GYRO;
    result_GYRO = rt_spi_bus_attach_device(&spi_dev_gyro, "gyro", "spi1", (void*)&gyro_cs_pin);
    if (result_GYRO == RT_EOK)
    {
        rt_kprintf("GYRO ATTACHED SUCCESS\n");
    }
    else
    {
        rt_kprintf("GYRO ATTACH FAILED\n");
    }
    rt_spi_configure(&spi_dev_gyro, &cfg_gyro);
    if (rt_device_find("gyro") != RT_NULL)
    {
        rt_kprintf("GYRO FOUND SUCCESS!\n");
    }
    else
    {
        rt_kprintf("GYRO NOT FOUND AFTER ATTACH\n");
    }
    rt_pin_mode(GYRO_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(GYRO_PIN, PIN_HIGH);

    imu_gyro_write(0x14,0xB6);rt_thread_mdelay(50);//Reset gyro
    imu_gyro_write(0x10,0x04);rt_thread_mdelay(50);//200Hz
    imu_gyro_write(0x0F,0x00);rt_thread_mdelay(50);//+-2000 dps

    rt_err_t result_ACCEL;
    result_ACCEL = rt_spi_bus_attach_device(&spi_dev_accel, "accel", "spi1", (void*)&accel_cs_pin);
    if (result_ACCEL == RT_EOK)
    {
        rt_kprintf("ACCEL ATTACHED SUCCESS\n");
    }
    else
    {
        rt_kprintf("ACCEL ATTACH FAILED\n");
    }
    rt_spi_configure(&spi_dev_accel, &cfg_accel);
    if (rt_device_find("accel") != RT_NULL)
    {
        rt_kprintf("ACCEL FOUND SUCCESS!\n");
    }
    else
    {
        rt_kprintf("ACCEL NOT FOUND AFTER ATTACH\n");
    }
    rt_pin_mode(ACCEL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ACCEL_PIN, PIN_HIGH);

    rt_pin_write(ACCEL_PIN, PIN_LOW);
    rt_thread_mdelay(1);
    rt_spi_send_then_recv(&spi_dev_accel, RT_NULL, 1, RT_NULL, 1);
    rt_thread_mdelay(10);
    rt_pin_write(ACCEL_PIN, PIN_HIGH);
    rt_thread_mdelay(1);
    rt_pin_write(ACCEL_PIN, PIN_LOW);
    rt_thread_mdelay(1);
    rt_spi_send_then_recv(&spi_dev_accel, RT_NULL, 1, RT_NULL, 1);
    rt_thread_mdelay(10);
    rt_pin_write(ACCEL_PIN, PIN_HIGH);
    rt_thread_mdelay(1);

    imu_accel_write(0x7E,0xB6);rt_thread_mdelay(80);//Reset accel
    imu_accel_write(0x7C,0x00);rt_thread_mdelay(80);//上电
    imu_accel_write(0x7D,0x04);rt_thread_mdelay(80);//启动
    imu_accel_write(0x40,0x0A);rt_thread_mdelay(80);//Normal mode
    imu_accel_write(0x40,0x0A);rt_thread_mdelay(80);//400Hz
    imu_accel_write(0x41,0x01);rt_thread_mdelay(80);//Set range to +-6g

    mahony_ahrs_IMU = rt_thread_create("mahony_ahrs_updateIMU",mahony_ahrs_updateIMU_thread_entry,RT_NULL,1024,20,5);
    if(mahony_ahrs_IMU!=RT_NULL)
    {
        rt_thread_startup(mahony_ahrs_IMU);
    }
    else
    {
        rt_kprintf("Failed to create Mahony Thread!\n");
    }

    Data_Send = rt_thread_create("Data_Send",data_send_thread_entry,RT_NULL,1024,15,5);
    
    if(Data_Send!=RT_NULL)
    {
        rt_thread_startup(Data_Send);
    }
    else
    {
        rt_kprintf("Failed to create Data_Send Thread!\n");
    }
    
    sensor_data_get = rt_thread_create("sensor_data_get",sensor_data_get_entry,RT_NULL,1024,25,5);
     if(sensor_data_get!=RT_NULL)
    {
        rt_thread_startup(sensor_data_get);
    }
    else
    {
        rt_kprintf("Failed to create Sensor_Data_Get Thread!\n");
    }
}
