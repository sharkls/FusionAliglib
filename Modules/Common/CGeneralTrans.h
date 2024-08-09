#ifndef CGENERALTRANS_H
#define CGENERALTRANS_H
#include <iostream>
#include <cmath>
class CGeneralTrans
{
public:
    //角度转换成弧度
    inline double FG_degree2rad(double degree)
    {
        return degree*M_PI/180.0;
    }
    
    //弧度转角度
    inline double FG_rad2degree(double rad)
    {
        return rad*180.0/M_PI;
    }

    //经纬度转换成墨卡托坐标
    inline bool LonLat2Mercator(double B, double L,double &mer_x, double &mer_y)
    {
        // f / * 扁率 * /, e / * 第一偏心率 * /, e_ / * 第二偏心率 * /, NB0 / * 卯酉圈曲率半径 * /, K, dtemp;
        // f, e, e_, NB0, K, dtemp=0
        // print('lonlat2mercator')
        double f = 0.0;
        double e = 0.0;
        double e_ = 0.0;
        double NB0 = 0.0;
        double E = 0.0;
        double dtemp = 0.0;
        E = float(exp(1));

        __B0 = B;
        __L0 = 0;
        if  (L < -M_PI || L > M_PI || B < (-M_PI / 2) || B > (M_PI / 2))
            return false;

        if (__A <= 0 || __B <= 0)
            return false;
        f = (__A - __B) / __A;

        dtemp = 1 - (__B / __A) * (__B / __A);
        if (dtemp < 0)
            return false;

        e = sqrt(dtemp);

        dtemp = (__A / __B) * (__A / __B) - 1;
        if (dtemp < 0)
            return false;
    
        e_ = sqrt(dtemp);
        NB0 = ((__A * __A) / __B) / sqrt(1 + e_ * e_ * cos(__B0) * cos(__B0));
        double K = NB0 * cos(__B0);
        mer_x = K * (L - __L0);
        mer_y = K * log(tan(M_PI / 4 + B / 2) * pow((1 - e * sin(B)) / (1 + e * sin(B)), e / 2));

        return true;
    }
    

    //墨卡托坐标转经纬度
    inline bool Mercator2LonLat(double B,double L,double X, double Y, double &Object_Long, double& Object_Lat )
    {
        // double f/*扁率*/, e/*第一偏心率*/, e_/*第二偏心率*/, NB0/*卯酉圈曲率半径*/, K, dtemp;
        // double E = exp(1);
        double f = 0.0;
        double e = 0.0;
        double e_ = 0.0;
        double NB0 = 0.0;
        double E = 0.0;
        double dtemp = 0.0;
        E = float(exp(1));
        __B0 = B;
        __L0 = 0;


        if (__A <= 0 || __B <= 0)
            return false;

        f = (__A - __B) / __A;
        dtemp = 1 - (__B / __A) * (__B / __A);
        if (dtemp < 0)
            return false;
        e = sqrt(dtemp);

        dtemp = (__A / __B) * (__A / __B) - 1;
        if (dtemp < 0)
            return false;
        e_ = sqrt(dtemp);
        NB0 = ((__A * __A) / __B) / sqrt(1 + e_ * e_ * cos(__B0) * cos(__B0));
        double K = NB0 * cos(__B0);
        Object_Long = FG_rad2degree(Y / K + __L0);
        B = 0.0;
        for(int i=0; i<__IterativeValue; i++)
        {
            B=M_PI/2-2*atan(pow(E, (-X/K)) * pow(E, (e/2)*log((1-e*sin(B))/(1+e*sin(B)))));
        }
            
        Object_Lat= FG_rad2degree(B);
    
        return true;
    }

    /*
    #   XYZ_To_BLH()：特征物体XY坐标转经纬度
    #   输入：
    #   WayPoint BLH_Origin: 激光雷达原始经纬度，角度
    #   WayPoint XYZ_Move: 特征物体XY坐标，米
    #   double rotaionangle： 激光雷达坐标系Y轴相对于正北方向夹角(0°～360°)顺时针
    #   输出：
    #   WayPoint *BLH_Move: 特征物体经纬度，角度
    #  */
    inline void XYZ_To_BLH(double original_long,double original_lat,double move_x, double move_y, double rotaionangle, double &Object_Long, double & Object_Lat)
    {
        double RadAngle =float(FG_degree2rad(rotaionangle));

        //激光器经纬度转墨卡托XY

        double mer_x, mer_y;
        LonLat2Mercator(FG_degree2rad(original_lat),FG_degree2rad(original_long), mer_x, mer_y);

        //坐标轴旋转到正北方向,计算Move点墨卡托XY坐标
        double mer_move_x = move_x * cos(RadAngle) + move_y * sin(RadAngle) + mer_x;
        double mer_move_y = move_y * cos(RadAngle) - move_x * sin(RadAngle) +mer_y;
        // double Object_Long,Object_Lat;
        Mercator2LonLat(FG_degree2rad(original_lat),FG_degree2rad(original_long),mer_move_y,mer_move_x, Object_Long, Object_Lat);
        // return Object_Long,Object_Lat  ; 
        return ;
    }

     /*
    #   BLH_To_XY()：特征物体XY坐标转经纬度
    #   输入：
    #   WayPoint BLH_Origin: 激光雷达原始经纬度，角度
    #   WayPoint XYZ_Move: 特征物体XY坐标，米
    #   double rotaionangle： 激光雷达坐标系Y轴相对于正北方向夹角(0°～360°)顺时针
    #   输出：
    #   WayPoint *BLH_Move: 特征物体经纬度，角度
    #  */
    inline void BLH_To_XYZ(float original_long,float original_lat,float &move_x, float &move_y, float rotaionangle, float Object_Long, float Object_Lat)
    {
        double RadAngle =float(FG_degree2rad(rotaionangle));
        //激光器经纬度转墨卡托XY
        double mer_x, mer_y;
        LonLat2Mercator(FG_degree2rad(original_lat),FG_degree2rad(original_long), mer_x, mer_y);

        double tar_x, tar_y;
        LonLat2Mercator(FG_degree2rad(Object_Lat),FG_degree2rad(Object_Long), tar_x, tar_y);

        auto l_fdX = tar_x - mer_x;
        auto l_fdY = tar_y - mer_y;


        move_x = l_fdX * cos(RadAngle) - l_fdY * sin(RadAngle);
        move_y = l_fdY * cos(RadAngle) + l_fdX * sin(RadAngle);
        return ;
    }


private:
    // 椭球体长半轴, 米
    float __A = 6378137;
    // 椭球体短半轴, 米
    float __B = 6356725;
    // float __B = 6356752.3142;

    // 标准纬度, 弧度（取激光雷达所在纬度）
    float __B0=0.0;
    // 原点经度, 弧度(0°)
    float __L0=0.0;
    //反向转换程序中的迭代初始值
    float __IterativeValue=10;
    float m_fdMathPi = 3.141592653589793;    //PI
};

#endif