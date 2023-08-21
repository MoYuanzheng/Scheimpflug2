#pragma once

#include<stdio.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<algorithm>
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>

using namespace std;
using namespace cv;
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
vector< cv::Point3d> P2_4_Corner;

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


//! ====================================>> 函数声明部分 <<=============================================

//! ====================================>> 点对函数部分 <<=============================================

//! 计算光心及传感器平面中心
//! 参数：无
//! 返回值：以沙姆点为原点坐标 ==> vector[0] -> 光心坐标    vector[1] -> 传感器中心坐标
std::vector< cv::Point3d> Calculate_Optical_Sensor_Center_Point();

//! 计算激光与传感器平面夹角
//! 参数：无
//! 返回值：夹角弧度
double Calculate_Laser_Sensor_Angle();

//! 通过sensor尺寸、角度计算传感器角点
//! 参数：
//!         Sensor_Center：传感器中心
//!         Sensor_Size    ：传感器尺寸（宽 * 长）
//!         theta_9            ：传感器平面与 y 轴的夹角
//! 返回值：无
void Calculate_P2_Corner(cv::Point3d Sensor_Center, cv::Size2f Sensor_Size, double theta_9);

//! 计算线面交点
//! 参数：
//!     optical_center ： 光心坐标
//!     P ：平面的法向量
//!     Line：直线的方向向量
//! 返回值：Line 与 P 相交的坐标点
cv::Point3d Calculate_Line_Plane_Intersection_Point(cv::Point3d optical_center, _Plane P, _Line Line);

//! 绕 x轴 像素顺时针旋转 theta 弧度
//! 参数：
//!         points：需要旋转的点
//!         theta：需要旋转的弧度值
//! 返回值：旋转后的结果
std::vector< std::vector< cv::Point3d>> Rotation(std::vector< std::vector< cv::Point3d>> points, double theta);

//! 计算激光平面的第一个取点位置
//! 参数：Cornor_Points：角点坐标
//! 返回值：第一个取点位置的世界坐标
std::vector< cv::Point3d> Calculate_P1_Pixel_Origin(std::vector< std::vector< cv::Point3d>> Cornor_Points);

//! 获取图像像素化的起始坐标
//! 参数：Cornor_Points：角点坐标
//! 返回值：像素坐标原点的世界坐标
cv::Point3d Calculate_P2_Pixel_Origin(std::vector< cv::Point3d> Cornor_Points);

//! 通过光心及sensor角点坐标计算Laser平面角点 
//! 参数：
//!         optical_center：光心
//!         plane：Laser平面法向量
//!         Special_Points：sensor角点坐标
//!   返回值：Laser平面角点 
std::vector< std::vector< cv::Point3d>> Calculate_Special_Points(
    cv::Point3d optical_center, std::vector<_Plane> plane, std::vector< cv::Point3d> Special_Points);

//! sensor平面像素化
//! 参数：
//!         points：sensor：平面的世界坐标点
//!         P2_Pixel_Origin：像素坐标原点
//! 返回值：被像素的坐标
std::vector< std::vector< cv::Point2d>> Coordinate_System_conversion_to_Pixel_P2(
    std::vector< std::vector< cv::Point3d>> points, cv::Point3d P2_Pixel_Origin);

//! 规则化生成点对
//! 参数：
//!         optical_center：光心坐标
//!         p1：Laser平面法向量
//!         p2：Sensor平面法向量
//!         P1_Origin：Laser平面第一个取点位置
//! 返回值：点对
//!        第一层Vec：图像序数
//!        第二层Vec
//!                     [0] ->激光平面上的点坐标
//!                     [1] ->传感平面上的点坐标
//!                     [2] ->以第一个点为原点的激光平面上的点坐标

std::vector< std::vector< std::vector< cv::Point3d>>> Regular_Generate_Point_Pair(
    cv::Point3d optical_center, std::vector<_Plane>p1, _Plane p2, std::vector< cv::Point3d> P1_Origin);

//! ====================================>> 标定函数部分 <<=============================================

//! 功 能：构建 P 矩阵，用于SVD分解的输入
//! 参数1：物体的世界坐标（自定义）
//! 参数2：图像像素坐标
//! 返回值：待分解矩阵P
cv::Mat Matrix_P(std::vector< cv::Point3d>Object, std::vector< cv::Point2d>Image);

//! 功 能：进行SVD分解，得到单应性矩阵 H
//! 参数1：由Object坐标与Image坐标构成的矩阵
//! 返回值：单应性矩阵H
cv::Mat Martix_H(cv::Mat Matrix_P);

//! 功 能：通过重投影 测试单应性矩阵 H，并返回 误差均值
//! 参数1：物体的世界坐标（自定义）
//! 参数2：图像像素坐标
//! 参数3：单应性矩阵 H
//! 参数4：_H_Reproject_Points : 各点重投影结果集合
//! 参数5：_H_Reproject_Errors : 各点重投影误差集合
//! 参数6：Normal_Flag : 单应性矩阵是否归一化
//! 返回值：重投影误差均值
double _H_Reproject(
    std::vector< cv::Point3d> Object,
    std::vector< cv::Point2d> Image,
    cv::Mat _H,
    std::vector< cv::Point2d>& _H_Reproject_Points,
    std::vector< cv::Point2d>& _H_Reproject_Errors,
    bool Normal_Flag);

void Normalize(const std::vector<cv::Point2d>& point_vec, std::vector<cv::Point2d>* normed_point_vec, cv::Mat* norm_T);
std::vector< cv::Mat> Remove_Normal_Martix_H(std::vector< cv::Mat> _H, std::vector< std::vector< cv::Point3d>>Object, std::vector< std::vector< cv::Point2d>>Image);

//! 求取内参
//! 参数1：单应性矩阵H
//! 返回值：内参矩阵
cv::Mat Solov_Camera_Internal_Parameter(std::vector<cv::Mat> _H);


//! 求取内参
//! 参数1：内参矩阵H
//! 参数1：单应性矩阵H
//! 返回值：外参矩阵
std::vector<cv::Mat> Solov_External_Parameter(cv::Mat _K1, std::vector < cv::Mat >_H);