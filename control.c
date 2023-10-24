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

/* 函数 AngleCalculate() 的主要作用：先将加速度数据除于灵敏度，将加速度数据由 LSB 转化成 g。注意变量类型使用 float 类型是为了保持精度，使用 atan2() 函数计算出角度（单位为弧度），
除以 3.14 再乘 180°，将单位转换成度。
将陀螺仪数据除于灵敏度，得到角速度，单位为 deg/s。
最后，将加速度初算的角度和陀螺仪角速度送入互补滤波器进行数据融合，得到最终稳定的精准小车倾角。*/
