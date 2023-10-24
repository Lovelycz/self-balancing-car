#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

extern unsigned char g_ucMainEventCount;
extern float g_fCarAngle;

void GetMpuData(void);//获取MPU数据
void AngleCalculate(void);//角度计算函数

#endif
