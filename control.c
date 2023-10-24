#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"

short x_nAcc,y_nAcc,z_nAcc;//加速度x轴、y轴、z轴数据
short x_nGyro,y_nGyro,z_nGyro;//陀螺仪x轴、y轴、z轴数据
float x_fAcc,y_fAcc,z_fAcc;

float g_fAccAngle;//加速度传感器经过atan2()解算得到的角度
float g_fGyroAngleSpeed;//陀螺仪角速度
float g_fCarAngle;//小车倾角
float dt = 0.005;//互补滤波器控制周期

unsigned char g_ucMainEventCount;//主事件计数，会用在中断中

void GetMpuData(void)//获取MPU-6050数据函数
{
    MPU_Get_Accelerometer(&x_nAcc,&y_nAcc,&z_nAcc);//获取MPU-6050加速度数据
    MPU_Get_Gyroscope(&x_nGyro,&y_nGyro,&z_nGyro); //获取MPU-6050陀螺仪数据
}

void AngleCalculate(void)//角度计算函数
{
    //-------加速度数据处理--------------------------
    //量程为±2g时，灵敏度：16384 LSB/g
    x_fAcc = x_nAcc / 16384.0;
    y_fAcc = y_nAcc / 16384.0;
    z_fAcc = z_nAcc / 16384.0;

    g_fAccAngle = atan2(y_fAcc,z_fAcc) * 180.0 / 3.14;

    //-------陀螺仪数据处理-------------------------
    //范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
    g_fGyroAngleSpeed = x_nGyro / 16.4;//计算角速度值                    

    //-------互补滤波---------------
    g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);
}


