#include "gps.h"

uint8_t recvBuffer[100];

//0: no data available, others: data available
uint8_t GPS_MsgRecv(uint8_t ch)
{
    static uint8_t stage = 0x00;
    static uint8_t cnt = 0;
    uint8_t ret = 0, clearFlag = 0;
    
    recvBuffer[cnt++]=ch;
    
    switch(stage)
    {
        case 0x00:      //'$'
            if(ch=='$')     stage++;
            break;
            
        case 0x01:      //'G'
            if(ch=='G')     stage++;
            else            clearFlag = 1;
            break;
            
        case 0x02:      //'P'/'A'
            if(ch=='P'||ch=='N')    stage++;
            else            clearFlag = 1;
            break;
            
        case 0x03:      //'R'/'G'
        case 0x04:      //'M'/'G'
        case 0x05:      //'C'/'A'
            stage++;
            break;
            
        case 0x06:      //Data
            if(ch=='*')     stage++;
            break;
        
        case 0x07:      //Checksum I
            stage++;
            break;
        
        case 0x08:      //Checksum II
            ret = cnt;
            clearFlag = 1;
            break;
    }
    
    if(clearFlag)
    {
        stage = 0x00;
        cnt = 0;
    }
        
    return ret;
}

void UTC2BTC(GPSMsgType* gps)
{
    #define isLeap(year) (((year%4==0&&year/100!=0)||(year%400==0)))
    static int day_list[2][12]={{31,28,31,30,31,30,31,31,30,31,30,31},{31,29,31,30,31,30,31,31,30,31,30,31}};
    gps->hour += 8;
    if(gps->hour > 23)
    {
        gps->hour -= 24;
        gps->day++;
        if(gps->day > day_list[isLeap(gps->year)][gps->month-1])
        {
            gps->day -= day_list[isLeap(gps->year)][gps->month-1];
            gps->month++;
            if(gps->month > 12)
            {
                gps->month -= 12;
                gps->year++;
            }
        }
    }
}

uint8_t GPS_Decode(char* buf, GPSMsgType* gpsm, uint8_t len)
{
    uint8_t tmp     = 0;
    uint8_t status  = 0;
    uint8_t result  = 0;
    //uint16_t checksum = buf[1];
    
    // Calculate CRC
    //for(int cnt=2;cnt<len-3;cnt++)
    //    checksum ^= buf[cnt];
    
    // Check CRC
    //if(checksum != HEX2OCT(buf[len-2])*16+HEX2OCT(buf[len-1]))  return 0;
    
    if(buf[0]=='\r'&&buf[1]=='\n')  buf=buf+2;  // Skip 0x0D and 0x0A
    
    if(!strncmp("$GNRMC",buf,6))
    {
        //printf("%s\r\n",buf);
        gpsm->hour = (buf[7] - '0') * 10 + (buf[8] - '0');
        gpsm->minute = (buf[9] - '0') * 10 + (buf[10] - '0');
        gpsm->second = (buf[11] - '0') * 10 + (buf[12] - '0');
        tmp = GetComma(buf,9);
        gpsm->day = (buf[tmp + 0] - '0') * 10 + (buf[tmp + 1] - '0');
        gpsm->month = (buf[tmp + 2] - '0') * 10 + (buf[tmp + 3] - '0');
        gpsm->year = (buf[tmp + 4] - '0') * 10 + (buf[tmp + 5] - '0') + 2000;
        UTC2BTC(gpsm);
        
        status = buf[GetComma(buf,2)];
        if(status=='A')
        {
            LatLonType gpswgs,gpsgcj,gpsbd;
            gpswgs.lat = DMM2Degree(GetDoubleNumber(&buf[GetComma(buf,3)])/100);
            gpswgs.lon = DMM2Degree(GetDoubleNumber(&buf[GetComma(buf,5)])/100);
            if(buf[GetComma(buf,4)] == 'S') gpswgs.lat = -gpswgs.lat;
            if(buf[GetComma(buf,6)] == 'W') gpswgs.lon = -gpswgs.lon;
            //gpsgcj = WGS2GCJ(gpswgs);
            //gpsbd = GCJ2BD(gpsgcj);
            gpsm->latitude = gpswgs.lat;
            gpsm->longitude = gpswgs.lon;
            result = 0x01;
        }
        else
        {
            gpsm->latitude = 0;
            gpsm->longitude = 0;
        }
    }
    
    else if(!strncmp("$GNGGA",buf,6))
    {
        gpsm->height=GetDoubleNumber(&buf[GetComma(buf,9)]);
    }

    return result;
}

uint8_t GetComma(char* str,uint8_t num)
{
    uint8_t cnt=0,tmp=0,comma=0;
    while(comma!=num)
    {
        tmp=str[cnt++];
        if(tmp=='\0') return 0;
        if(tmp==',') comma++;
    }
    return cnt;
}

double GetDoubleNumber(char *s)
{
    char buf[128];
    int i;
    double rev;
    i=GetComma(s,1);
    strncpy(buf,s,i);
    buf[i]='\0';
    rev=atof(buf);

    return rev;
}

double DMM2Degree(double dmm)
{
    return (int)dmm + (dmm - (int)dmm)/60.0*100.0;
}


LatLonType WGS2GCJ(LatLonType wgsll)
{
    if(outOfChina(wgsll))
    {
        return wgsll;
    }
    LatLonType gcjll;
    LatLonType deltaD = delta(wgsll.lat,wgsll.lon);
    gcjll.lat = wgsll.lat + deltaD.lat;
    gcjll.lon = wgsll.lon + deltaD.lon;
    return gcjll;
}

LatLonType GCJ2WGS(LatLonType gcjll)
{
    if(outOfChina(gcjll))
    {
        return gcjll;
    }
    LatLonType wgsll;
    LatLonType deltaD = delta(gcjll.lat,gcjll.lon);
    wgsll.lat = gcjll.lat - deltaD.lat;
    wgsll.lon = gcjll.lon - deltaD.lon;
    return wgsll;
}

LatLonType GCJ2BD(LatLonType gcjll)
{
    LatLonType bdll;
    double x = gcjll.lon, y = gcjll.lat;  
    double z = sqrt(x * x + y * y) + 0.00002 * sin(y * X_PI);  
    double theta = atan2(y, x) + 0.000003 * cos(x * X_PI);  
    bdll.lat = z * sin(theta) + 0.006;
    bdll.lon = z * cos(theta) + 0.0065;  
    return bdll;
}

LatLonType BD2GCJ(LatLonType bdll)
{
    LatLonType gcjll;
    double x = bdll.lon - 0.0065;
    double y = bdll.lat - 0.006;  
    double z = sqrt(x * x + y * y) - 0.00002 * sin(y * X_PI);  
    double theta = atan2(y, x) - 0.000003 * cos(x * X_PI);  
    gcjll.lat = z * sin(theta);
    gcjll.lon = z * cos(theta);  
    return gcjll;
}
LatLonType WGS2BD(LatLonType wgsll)
{
    return GCJ2BD(WGS2GCJ(wgsll));
}
LatLonType BD2WGS(LatLonType bdll)
{
    return GCJ2WGS(BD2GCJ(bdll));
}


int outOfChina(LatLonType ll)
{
    if (ll.lon < 72.004 || ll.lon > 137.8347)  return 1;  
    if (ll.lat < 0.8293 || ll.lat > 55.8271)   return 1;  
    return 0;
}

double transformLat(double x, double y)
{  
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(fabs(x));  
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;  
    ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;  
    ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;  
    return ret;  
}  

double transformLon(double x, double y)
{  
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(fabs(x));  
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;  
    ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;  
    ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;  
    return ret;  
}

LatLonType delta(double wgLat, double wgLon)
{
     LatLonType latlng;
     double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);  
     double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);  
     double radLat = wgLat / 180.0 * PI;  
     double magic = sin(radLat);  
     magic = 1 - OFFSET * magic * magic;  
     double sqrtMagic = sqrt(magic);  
     dLat = (dLat * 180.0) / ((AXIS * (1 - OFFSET)) / (magic * sqrtMagic) * PI);  
     dLon = (dLon * 180.0) / (AXIS / sqrtMagic * cos(radLat) * PI);  
     latlng.lat = dLat;
     latlng.lon = dLon;
     return latlng;
}
