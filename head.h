#pragma once

#include<stdio.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<algorithm>
#include<opencv2/opencv.hpp>

using namespace std;

const double PI = 4 * atan(1);
//! 平面结构
struct _Plane {
    double A;
    double B;
    double C;
    double D;
};
//! 直线结构
struct _Line
{
    double a;
    double b;
    double c;
};

//! 角度 -> 弧度制 -> 单位rad
//! theta_1[已知] 激光平面与光轴夹角
const double theta_1 = 45.0 / 180 * PI;

//! theta_2[未知] 光轴与传感器平面夹角
double theta_2 = 0;

//! theta_3[未知] 激光平面与传感器平面夹角
double theta_3 = 0;

//! theta_4[未知] 激光平面与镜头平面夹角
double theta_4 = PI / 2 - theta_1;

//! theta_9[未知] 坐标系旋转角度
double theta_9 = 0;


//! 长度 -> 单位毫米
//! a[已知] 为激光平面至镜头光心的距离
//! b[已知] 为镜头光心至传感器的距离
const double distance_a = 50.0, distance_b = 35.0;
const double distance_e = distance_a * tan(theta_1);
const double distance_f = distance_b * cos(PI / 2 - theta_1);
const double distance_k = distance_b * sin(PI / 2 - theta_1);
double distance_h = distance_e - distance_k;
double distance_g = distance_a / tan(theta_1);

//! p1为激光平面
std::vector<_Plane> P1;

//! 靶面 -> 尺寸 -> 单位毫米
const cv::Size2f Sensor_Size = { 4.608,9.216 };

//! 靶面 -> 坐标 -> 角点
vector<cv::Point3f> P2_4_Corner;

//! 靶面 -> 尺寸 -> 像素
const cv::Size Image_Size = { 1024,2048 };

//! p2为成像平面（传感器平面）
_Plane P2 = { 0.0,0.0,0.0,0.0 };

//! 生成直线数量
const int LINE_NUMBER = 15 * 11;

//! 设置畸变因子
//const double k1 = 0.04408749451738147;
//const double k2 = 0.003288813627739166;
//const double k3 = 0.0008529518568999;

//! 设置内参
//! x轴与y轴 每毫米对应的采样点

const double Pixel_length = 0.0045;

const double Rows_Sample_Rate = 1 / Pixel_length;
const double Cols_Sample_Rate = 1 / Pixel_length;


//! ===============================================================================================

//! 函数声明部分
//! 
//! 点对函数部分
std::vector<cv::Point3f> Calculate_Optical_Sensor_Center_Point();

double Calculate_Laser_Sensor_Angle();

void Calculate_P2_Corner(cv::Point3f Sensor_Center, cv::Size2f Sensor_Size, double theta_9);

cv::Point3f Calculate_Line_Plane_Intersection_Point(cv::Point3f optical_center, _Plane P, _Line Line);
std::vector< std::vector<cv::Point3f>> Rotation(std::vector< std::vector<cv::Point3f>> points, double theta);
std::vector < cv::Point3f> Calculate_P1_Pixel_Origin(std::vector < std::vector<cv::Point3f>> Cornor_Points);
cv::Point3f Calculate_P2_Pixel_Origin(std::vector<cv::Point3f> Cornor_Points);
//! 两平面特殊点 
std::vector< std::vector<cv::Point3f>> Calculate_Special_Points(cv::Point3f optical_center, std::vector<_Plane> plane, std::vector<cv::Point3f> Special_Points);
std::vector< std::vector<cv::Point2f>> Coordinate_System_conversion_to_Pixel_P2(std::vector < std::vector<cv::Point3f>> points, cv::Point3f P2_Pixel_Origin);
std::vector<cv::Point3f> FOV_Points_Small(vector<cv::Point3f> Corner_Points);

std::vector < std::vector<vector<cv::Point3f>>> Regular_Generate_Point_Pair(cv::Point3f optical_center, _Plane p2, std::vector < cv::Point3f> P1_Origin);

//! 标定函数部分 ===============================================================================================
//! 功 能：构建 P 矩阵，用于SVD分解的输入
//! 参数1：物体的世界坐标（自定义）
//! 参数2：图像像素坐标
cv::Mat Matrix_P(std::vector<cv::Point3f>Object, std::vector<cv::Point2f>Image);

//! 功 能：进行SVD分解，得到单应性矩阵 H
//! 参数1：由Object坐标与Image坐标构成的矩阵
cv::Mat Martix_H(cv::Mat Matrix_P);

//! 功 能：通过重投影 测试单应性矩阵 H，并返回 误差均值
//! 参数1：物体的世界坐标（自定义）
//! 参数2：图像像素坐标
//! 参数3：单应性矩阵 H
//! 参数4：_H_Reproject_Points : 各点重投影结果集合
//! 参数5：_H_Reproject_Errors : 各点重投影误差集合
//! 参数6：Normal_Flag : 单应性矩阵是否归一化
double _Reproject(
    std::vector<cv::Point3f> Object,
    std::vector<cv::Point2f> Image,
    cv::Mat _H,
    std::vector<cv::Point2f>& _H_Reproject_Points,
    std::vector<cv::Point2f>& _H_Reproject_Errors,
    bool Normal_Flag);