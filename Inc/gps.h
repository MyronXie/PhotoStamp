/**
  ******************************************************************************
  * @file           : gps.h
  * @brief          : Driver for GPS
  ******************************************************************************
  * @Version        : 1.4(200507)
  * @Author         : Myron Xie
  ******************************************************************************
  */

#ifndef __GPS_H
#define __GPS_H

#include "main.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

typedef struct
{
    uint16_t    year;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     minute;
    uint8_t     second;
    double      latitude;
    double      longitude;
    float       height;
}GPSMsgType;

typedef struct
{
    double  lat;
    double  lon;
}LatLonType;

#define PI      3.14159265358979324
#define AXIS    6378245.0
#define OFFSET  0.00669342162296594323
#define X_PI    PI*3000.0/180.0

#define HEX2OCT(x) (x<'0'?0:(x<='9'?x-'0':(x<'A'?0:(x<='F'?x-'A'+10:0))))
#define ISGPSMSG(x)   ((x>='0'&&x<='9')||(x>='A'&&x<='Z')||(x=='$')||(x==',')||(x=='.')||(x=='*')||(x=='-'))

extern uint8_t recvBuf[100];

uint8_t GPS_MsgRecv(uint8_t ch);
uint8_t GPS_Decode(char* buf, GPSMsgType* gpsm, uint8_t len);
uint8_t GetComma(char* str,uint8_t num);
double GetDoubleNumber(char *s);
double DMM2Degree(double dmm);
void UTC2BJT(GPSMsgType* gps);

double transformLat(double x, double y);
double transformLon(double x, double y);
LatLonType delta(double wgLat, double wgLon);
int outOfChina(LatLonType ll);

LatLonType WGS2GCJ(LatLonType wgsll);
LatLonType GCJ2WGS(LatLonType gcjll);
LatLonType GCJ2BD(LatLonType gcjll);
LatLonType BD2GCJ(LatLonType bdll);
LatLonType WGS2BD(LatLonType wgsll);
LatLonType BD2WGS(LatLonType bdll);

#endif /*__GPS_H */


/****END OF FILE****/
