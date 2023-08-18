#include"head.h"

//! 以激光平面与成像平面与镜头平面的交点作为空间直角坐标系原点
//! 激光平面垂直与y轴且过原点
int main() {

    //! 求 光心位置 和 像平面原点
    std::vector<cv::Point3f> point_center = Calculate_Optical_Sensor_Center_Point();

    //! 提取 光心位置 
    cv::Point3f Optocal_Center = point_center[0];
    //! 提取 传感器中心
    cv::Point3f Sensor_Center = point_center[1];

    //! 求传感器与激光夹角
    theta_3 = Calculate_Laser_Sensor_Angle();
    theta_9 = PI / 2 - theta_3;
    //! 计算传感器平面方程
    P2 = { 0,0 * cos(theta_9) + (-1) * (-sin(theta_9)),0 * sin(theta_9) + (-1) * cos(theta_9),0 };

    //! 计算 P2 角点
    Calculate_P2_Corner(Sensor_Center, Sensor_Size, theta_9);

    //! 写入 P1 平面，多个
    P1.push_back({ 0,1.0,0,0 });

    //! 得到 P1 旋转后的世界坐标原点
    cv::Point3f P2_Pixel_Origin = Calculate_P2_Pixel_Origin(P2_4_Corner);

    //! 计算 P1 角点
    std::vector<std::vector<cv::Point3f>> P1_4_Corner = Calculate_Special_Points(Optocal_Center, P1, P2_4_Corner);

    //! 获得 P1 第一个标定交叉点位置（世界坐标）
    std::vector<cv::Point3f> P1_Origin = Calculate_P1_Pixel_Origin(P1_4_Corner);

    //! 求 P1 与 P2 的映射点对以及 P1 平面的交叉点坐标（以第一个交叉点为原点）
    std::vector< std::vector<vector<cv::Point3f>>> PointPair = Regular_Generate_Point_Pair(Optocal_Center, P2, P1_Origin);

    //! 提取部分点 sensor交点，laser交叉点
    std::vector<vector<cv::Point3f>> Image_Point_Pair;
    std::vector<vector<cv::Point3f>> Laser_Point_Pair;
    for (int i = 0; i < PointPair.size(); i++) {
        Image_Point_Pair.push_back(PointPair[i][1]);
        Laser_Point_Pair.push_back(PointPair[i][2]);
    }

    //! 改变像平面坐标系 以像平面角点为原点 建立像素坐标
    std::vector < std::vector<cv::Point2f>>Image_Pixel_Points = Coordinate_System_conversion_to_Pixel_P2(Image_Point_Pair, P2_Pixel_Origin);
    ////! 根据matlab格式打印坐标
    //cout << endl << "p1点 x" << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p1[i].x << ' ';
    //}
    //cout << endl << "p1点 y" << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p1[i].y << ' ';
    //}
    //cout << endl << "p1点 z" << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p1[i].z << ' ';
    //}
    //cout << endl << "p2点 x" << endl;
    //for (int i = 0; i < PointPair.p2.size(); i++) {
    //    cout << PointPair.p2[i].x << ' ';
    //}
    //cout << endl << "p2点 y" << endl;
    //for (int i = 0; i < PointPair.p2.size(); i++) {
    //    cout << PointPair.p2[i].y << ' ';
    //}
    //cout << endl << "p2点 z" << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p2[i].z << ' ';
    //}
    //cout << endl << "p2像素点 x" << endl;
    //for (int i = 0; i < PointPair.p2.size(); i++) {
    //    cout << Laser_Pixel_Points[i].x << ' ';
    //}
    //cout << endl << "p2像素点 y" << endl;
    //for (int i = 0; i < PointPair.p2.size(); i++) {
    //    cout << Laser_Pixel_Points[i].y << ' ';
    //}
    //cout << endl << "p2像素点 z" << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << Laser_Pixel_Points[i].z << ' ';
    //}
    //cout << endl << "平移像素 x" << endl;
    //for (int i = 0; i < Image_Pixel_Points.size(); i++) {
    //    cout << Image_Pixel_Points[i].x << ' ';
    //}
    //cout << endl << "平移像素 y" << endl;
    //for (int i = 0; i < Image_Pixel_Points.size(); i++) {
    //    cout << Image_Pixel_Points[i].y << ' ';
    //}

    //std::vector<cv::Point3f>Object = {
    //    {0.00000000, 0.00000000 , 0.00000000 },
    //    {0.00000000, 10.0000000 , 0.00000000 },
    //    {0.00000000, 20.0000000 , 0.00000000 },
    //    {0.00000000, 30.0000000 , 0.00000000 },
    //    {10.0000000, 0.00000000 , 0.00000000 },
    //    {10.0000000, 10.0000000 , 0.00000000 },
    //    {10.0000000, 20.0000000 , 0.00000000 },
    //    {10.0000000, 30.0000000 , 0.00000000 },
    //    {20.0000000, 0.00000000 , 0.00000000 },
    //    {20.0000000, 10.0000000 , 0.00000000 },
    //    {20.0000000, 20.0000000 , 0.00000000 },
    //    {20.0000000, 30.0000000 , 0.00000000 },
    //    {30.0000000, 0.00000000 , 0.00000000 },
    //    {30.0000000, 10.0000000 , 0.00000000 },
    //    {30.0000000, 20.0000000 , 0.00000000 },
    //    {30.0000000, 30.0000000 , 0.00000000 },
    //    {40.0000000, 0.00000000 , 0.00000000 },
    //    {40.0000000, 10.0000000 , 0.00000000 },
    //    {40.0000000, 20.0000000 , 0.00000000 },
    //    {40.0000000, 30.0000000 , 0.00000000 },
    //    {50.0000000, 0.00000000 , 0.00000000 },
    //    {50.0000000, 10.0000000 , 0.00000000 },
    //    {50.0000000, 20.0000000 , 0.00000000 },
    //    {50.0000000, 30.0000000 , 0.00000000 }
    //};
    //std::vector<cv::Point2f>Image = {
    //    {424.833801,86.0681152 },
    //    {423.785706,130.357147 },
    //    {422.500000,172.500000 },
    //    {421.000000,213.500000 },
    //    {388.000000,100.000000 },
    //    {387.750000,144.125000 },
    //    {386.375000,186.375000 },
    //    {384.681824,227.681824 },
    //    {348.758057,113.338707 },
    //    {348.625000,158.625000 },
    //    {348.750000,201.125000 },
    //    {348.500000,243.000000 },
    //    {308.785706,127.357140 },
    //    {309.241943,172.661285 },
    //    {310.000000,216.500000 },
    //    {310.000000,258.500000 },
    //    {267.166656,141.722229 },
    //    {268.214294,187.642853 },
    //    {269.420013,232.139999 },
    //    {270.500000,274.500000 },
    //    {222.927689,156.327515 },
    //    {225.419998,203.139999 },
    //    {228.000000,247.750000 },
    //    {228.714508,290.991791 }
    //};

        //! 手动标定部分
    //! -------- OpenCV 单应性矩阵测试 ----------------------------------------------------------------------------------------------------
    std::vector< std::vector<cv::Point2f>> _CV_H_Reproject_Points;
    std::vector< std::vector<cv::Point2f>> _CV_H_Reproject_Errors;
    std::vector< cv::Mat> Homography_Matrix;
    cv::Mat xxx = cv::findHomography(Laser_Point_Pair[0], Image_Pixel_Points[0], cv::LMEDS);
    std::vector<double> _CV_H_Error_Mean;
    for (int i = 0; i < Laser_Point_Pair.size(); i++) {
        cv::Mat Homegraphy = cv::findHomography(Laser_Point_Pair[i], Image_Pixel_Points[i], cv::LMEDS);
        Homegraphy.convertTo(Homegraphy, CV_32FC1);
        Homography_Matrix.push_back(Homegraphy);

        _CV_H_Reproject_Points.push_back({});
        _CV_H_Reproject_Errors.push_back({});
        _CV_H_Error_Mean.push_back(_Reproject(Laser_Point_Pair[i], Image_Pixel_Points[i], Homegraphy, _CV_H_Reproject_Points[i], _CV_H_Reproject_Errors[i], false));
    }

    //! --------SVD 单应性矩阵测试----------------------------------------------------------------------------------------------------
    std::vector< std::vector<cv::Point2f>> _H_Reproject_Points;
    std::vector< std::vector<cv::Point2f>> _H_Reproject_Errors;
    std::vector<cv::Mat> _H;
    std::vector<double> _H_Error_Mean;
    for (int i = 0; i < Laser_Point_Pair.size(); i++) {
        cv::Mat _P = Matrix_P(Laser_Point_Pair[i], Image_Pixel_Points[i]);
        _H.push_back(Martix_H(_P));

        _H_Reproject_Points.push_back({});
        _H_Reproject_Errors.push_back({});
        _H_Error_Mean.push_back({ _Reproject(Laser_Point_Pair[i], Image_Pixel_Points[i], _H[i], _H_Reproject_Points[i], _H_Reproject_Errors[i], true) });
    }

    // !-------- OpenCV 自动标定 ----------------------------------------------------------------------------------------------------
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecsMat, tvecsMat;

    //! cv::CALIB_TILTED_MODEL 
    float err_first = cv::calibrateCamera(Laser_Point_Pair, Image_Pixel_Points, Image_Size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, cv::CALIB_TILTED_MODEL);

    // !**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--
    //! 保存重新计算得到的投影点
    std::vector<std::vector<cv::Point2f>> Reprojection_Points;
    std::vector<std::vector<cv::Point2f>> Reprojection_Error_All;
    Reprojection_Error_All.resize(10);
    std::vector<double> Reprojection_Error_Mean;
    Reprojection_Points.push_back({});
    projectPoints(Laser_Point_Pair[0], rvecsMat[0], tvecsMat[0], cameraMatrix, distCoeffs, Reprojection_Points[0]);
    cout << "test" << endl;
    for (int i = 0; i < Image_Pixel_Points.size(); i++) {
        //Reprojection_Error_All.push_back({});
        for (int j = 0; j < Image_Pixel_Points[i].size(); j++) {
            Reprojection_Error_All[i].push_back({
                fabs(Image_Pixel_Points[i][j].x - Reprojection_Points[i][j].x),
                fabs(Image_Pixel_Points[i][j].y - Reprojection_Points[i][j].y),
                });
        }

        Reprojection_Error_Mean.push_back(cv::mean(Reprojection_Error_All[i])[0]);
    }

    /*输出内参数*/
    std::cout << "cameraMatrix:" << std::endl << cameraMatrix << std::endl;
    std::cout << "distCoeffs:" << std::endl << distCoeffs << std::endl;
    std::cout << "rvecsMat:" << std::endl << rvecsMat[0] << std::endl;
    std::cout << "tvecsMat:" << std::endl << tvecsMat[0] << std::endl;
    std::cout << "err_first:" << std::endl << err_first << std::endl;
    return 0;
}

//! 求取光心及传感器中心坐标
std::vector<cv::Point3f> Calculate_Optical_Sensor_Center_Point() {

    //! P[0]为光心 P[1]为像原点
    std::vector<cv::Point3f> P;

    //! 光心的坐标为（0, -a, -e）
    //! 传感器中心为(0,-(a+f),-(e-k) )
    //! 镜头光心坐标
    cv::Point3f Optical_Center = { 0.0, -static_cast<float>(distance_a), -static_cast<float>(distance_e) };

    P.push_back(Optical_Center);

    cv::Point3f Sensor_Center = { 0.0, -static_cast<float>((distance_a + distance_f)) ,-static_cast<float>(distance_h) };

    P.push_back(Sensor_Center);

    return P;
}

//! 计算传感器与激光平面夹角 theta_9
double Calculate_Laser_Sensor_Angle() {

    return PI / 2 - acos((distance_a + distance_b * cos(theta_1)) / sqrt(pow((distance_a - distance_b / (sqrt(2))), 2) + pow((distance_a + distance_b / (sqrt(2))), 2)));

}

//! 计算 Sensor 角点
void Calculate_P2_Corner(cv::Point3f Sensor_Center, cv::Size2f Sensor_Size, double theta_9) {
    P2_4_Corner.push_back({
        -Sensor_Size.height / 2,
        Sensor_Center.y + static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
        Sensor_Center.z + static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
    P2_4_Corner.push_back({
        +Sensor_Size.height / 2,
        Sensor_Center.y + static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
        Sensor_Center.z + static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
    P2_4_Corner.push_back({
        -Sensor_Size.height / 2,
        Sensor_Center.y - static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
        Sensor_Center.z - static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
    P2_4_Corner.push_back({
        +Sensor_Size.height / 2,
        Sensor_Center.y - static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
        Sensor_Center.z - static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
}

//! 求线面交点 - 世界坐标
cv::Point3f Calculate_Line_Plane_Intersection_Point(cv::Point3f optical_center, _Plane P, _Line Line) {

    cv::Point3f point = { 0.0,0.0,0.0 };

    point.x =
        optical_center.x - Line.a * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);
    point.y =
        optical_center.y - Line.b * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);
    point.z =
        optical_center.z - Line.c * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);

    return point;
}

//! 计算 FOV 视场坐标（梯形视场中的最大矩形 X为长边） => 得到激光平面  自己世界坐标原点
std::vector<cv::Point3f> FOV_Points_Small(vector<cv::Point3f> Corner_Points) {

    //! 得直线方程
    vector<cv::Point3f> Corner_Small;

    //! X轴坐标改为绝对值较小的一个
    double x_small = fabs(Corner_Points[0].x);
    for (int i = 1; i < Corner_Points.size(); i++) {
        if (x_small >= fabs(Corner_Points[i].x)) {
            x_small = fabs(Corner_Points[i].x);
        }
    }

    double x = 0.0;
    for (int i = 0; i < Corner_Points.size(); i++) {
        if (Corner_Points[i].x >= 0) {
            x = x_small;
        }
        else {
            x = -x_small;
        }
        Corner_Small.push_back({
               static_cast<float>(x),
               Corner_Points[i].y,
               Corner_Points[i].z
            });
    }
    return Corner_Small;
}

//! 激光平面 第一个交叉点
std::vector < cv::Point3f> Calculate_P1_Pixel_Origin(std::vector < std::vector<cv::Point3f>> Cornor_Points) {
    //! 应返回 
    //! x -> 绝对值最小 （p1:x -> 绝对值最小 p2:x -> 绝对值最大 由于p2角点的x绝对值都相等 故返回绝对值最小）
    //! y -> 值最大
    //! z -> 值最大
    std::vector<cv::Point3f> P1_Origin;
    for (int i = 0; i < Cornor_Points.size(); i++) {
        double x_min = Cornor_Points[i][0].x;
        double y_min = Cornor_Points[i][0].y;
        double z_max = Cornor_Points[i][0].z;

        for (int j = 1; j < Cornor_Points[i].size(); j++) {
            if (x_min >= fabs(Cornor_Points[i][j].x)) {
                x_min = fabs(Cornor_Points[i][j].x);
            }

            if (y_min <= Cornor_Points[i][j].y) {
                y_min = Cornor_Points[i][j].y;
            }

            if (z_max <= Cornor_Points[i][j].z) {
                z_max = Cornor_Points[i][j].z;
            }
        }
        P1_Origin.push_back({ static_cast<float>(x_min), static_cast<float>(y_min), static_cast<float>(z_max) });
    }
    return P1_Origin;
}

//! 旋转后的像素平面 以x最大 y最小的点为原点
cv::Point3f Calculate_P2_Pixel_Origin(std::vector<cv::Point3f> Cornor_Points) {
    std::vector< std::vector<cv::Point3f>> Cornor_Points_V2;
    Cornor_Points_V2.push_back(Cornor_Points);
    std::vector< std::vector<cv::Point3f>> Origin = Rotation(Cornor_Points_V2, theta_9);

    return  Origin[0][1];
}

//! 两平面特殊点 √
std::vector< std::vector<cv::Point3f>> Calculate_Special_Points(cv::Point3f optical_center, std::vector<_Plane> plane, std::vector<cv::Point3f> Special_Points) {
    std::vector< std::vector<cv::Point3f>> Another_Points;

    for (int i = 0; i < plane.size(); i++) {
        Another_Points.push_back({});
        for (int j = 0; j < Special_Points.size(); j++) {
            Another_Points[i].push_back(
                Calculate_Line_Plane_Intersection_Point(
                    optical_center, plane[i], {
                        Special_Points[j].x - optical_center.x,
                        Special_Points[j].y - optical_center.y,
                        Special_Points[j].z - optical_center.z
                    }
            ));
        }
    }
    return Another_Points;
}

//! 像素逆时针旋转 theta 弧度
std::vector< std::vector<cv::Point3f>> Rotation(std::vector< std::vector<cv::Point3f>> points, double theta) {

    std::vector < std::vector<cv::Point3f>> points_R;
    for (int i = 0; i < points.size(); i++) {

        points_R.push_back({});
        for (int j = 0; j < points[i].size(); j++) {

            points_R[i].push_back({ points[i][j].x,0.0,0.0 });

            //! 旋转 交点逆时针旋转 PI/2 - theta_9 个弧度
            points_R[i][j].y = points[i][j].y * cos(theta_9) + points[i][j].z * sin(theta_9);
            points_R[i][j].z = -points[i][j].y * sin(theta_9) + points[i][j].z * cos(theta_9);
        }
    }
    return points_R;
}

//! 将传感器平面转为以角点建立坐标系（旋转、平移及像素化）
std::vector< std::vector<cv::Point2f>> Coordinate_System_conversion_to_Pixel_P2(std::vector < std::vector<cv::Point3f>> points, cv::Point3f P2_Pixel_Origin) {

    std::vector < std::vector<cv::Point3f>> points_R = Rotation(points, theta_9);

    //! 平移至角点 建立像素坐标系
    std::vector < std::vector<cv::Point3f>> points_RT;
    for (int i = 0; i < points_R.size(); i++) {
        points_RT.push_back({});
        for (int j = 0; j < points_R[i].size(); j++)
        {
            if (points_R[i][j].z < 1e-3) {
                points_R[i][j].z = 0.0;
            }

            points_RT[i].push_back({
               -(points_R[i][j].y - P2_Pixel_Origin.y),
               -(points_R[i][j].x - P2_Pixel_Origin.x),
                +points_R[i][j].z
                }
            );
        }

    }

    //! 转像素  按照采样比进行像素转换
    std::vector < vector<cv::Point2f>> Image_Pixel_Points;
    for (int i = 0; i < points_RT.size(); i++) {
        Image_Pixel_Points.push_back({});
        for (int j = 0; j < points_RT[i].size(); j++) {
            Image_Pixel_Points[i].push_back({
                points_RT[i][j].x * static_cast<float>(Cols_Sample_Rate) ,
                points_RT[i][j].y * static_cast<float>(Rows_Sample_Rate)
                }
            );
        }
    }

    return Image_Pixel_Points;
}

//! 求取点对 p1 -> p2 映射，并保留 P1 映射点的坐标系坐标
std::vector < std::vector<vector<cv::Point3f>>> Regular_Generate_Point_Pair(cv::Point3f optical_center, _Plane p2, std::vector < cv::Point3f> P1_Origin) {
    //! PointPair[0] => P1平面
    //! PointPair[1] => P2平面
    //! PointPair[2] => P1平面的交叉点坐标
    //! 每一毫米生成一个点
    vector<cv::Point3f> PointPair_0;
    vector<cv::Point3f> PointPair_1;
    vector<cv::Point3f> PointPair_2;
    std::vector<std::vector<vector<cv::Point3f>>> PointPair;

    int count = 0;
    float _x, _y, _z;
    for (int k = 0; k < P1_Origin.size(); k++) {
        PointPair.push_back({});
        for (float i = 0.0; i < 17; i++) {

            for (float j = 0.0; j < 12; j++)
            {
                _x = P1_Origin[k].x - i;
                _y = 0.0;
                _z = P1_Origin[k].z - j;
                PointPair_0.push_back({ _x,_y,_z });

                PointPair_1.push_back(Calculate_Line_Plane_Intersection_Point(
                    optical_center, p2, {
                        _x - optical_center.x,
                        _y - optical_center.y,
                        _z - optical_center.z
                    }));
                PointPair_2.push_back({ j, i, 0 });
            }
        }
        PointPair[k].push_back(PointPair_0);
        PointPair[k].push_back(PointPair_1);
        PointPair[k].push_back(PointPair_2);
        PointPair_0.clear();
        PointPair_1.clear();
        PointPair_2.clear();
    }

    return PointPair;
}

//! 标定函数部分
//! 构建初始矩阵P
cv::Mat Matrix_P(std::vector<cv::Point3f>Object, std::vector<cv::Point2f>Image) {

    int row_number = Object.size();

    //! 认定激光平面深度为0 故Z轴不参与计算 
    cv::Mat Matrix_P = cv::Mat::zeros(row_number * 2, 9, CV_32F);
    int index = 0;

    for (int i = 0; i < row_number * 2; i++) {
        //! 偶数列
        index = i / 2;
        if (i % 2 == 0) {
            Matrix_P.at<float>(i, 0) = Object[index].x;
            Matrix_P.at<float>(i, 1) = Object[index].y;
            //Matrix_P.at<float>(i, 2) = Object[index].z;
            Matrix_P.at<float>(i, 2) = 1.0;

            Matrix_P.at<float>(i, 3) = 0.0;
            Matrix_P.at<float>(i, 4) = 0.0;
            //Matrix_P.at<float>(i, 6) = 0.0;
            Matrix_P.at<float>(i, 5) = 0.0;

            Matrix_P.at<float>(i, 6) = -Image[index].x * Object[index].x;
            Matrix_P.at<float>(i, 7) = -Image[index].x * Object[index].y;
            //Matrix_P.at<float>(i, 10) = -Image[index].x * Object[index].z;
            Matrix_P.at<float>(i, 8) = -Image[index].x * 1.0;
        }
        //! 奇数列
        else {
            Matrix_P.at<float>(i, 0) = 0.0;
            Matrix_P.at<float>(i, 1) = 0.0;
            //Matrix_P.at<float>(i, 2) = 0.0;
            Matrix_P.at<float>(i, 2) = 0.0;

            Matrix_P.at<float>(i, 3) = Object[index].x;
            Matrix_P.at<float>(i, 4) = Object[index].y;
            //Matrix_P.at<float>(i, 6) = Object[index].z;
            Matrix_P.at<float>(i, 5) = 1.0;

            Matrix_P.at<float>(i, 6) = -Image[index].y * Object[index].x;
            Matrix_P.at<float>(i, 7) = -Image[index].y * Object[index].y;
            //Matrix_P.at<float>(i, 10) = -Image[index].y * Object[index].z;
            Matrix_P.at<float>(i, 8) = -Image[index].y * 1.0;
        }
    }
    return Matrix_P;
}

//! 通过对 P 进行 SVD 分解得到单应性矩阵 H
cv::Mat Martix_H(cv::Mat Matrix_P) {
    //cv::Mat Martix_H = cv::Mat::zeros(9, 1, CV_32F);

    // 创建cv::SVD对象
    cv::SVD svd;

    // 进行SVD分解
    svd(Matrix_P);

    // 获取分解后的结果
    //cv::Mat U = svd.u;      // 正交矩阵U
    //cv::Mat W = svd.w;      // 对角矩阵W
    cv::Mat Vt = svd.vt;    // 转置正交矩阵Vt

    // 输出分解结果
    //std::cout << "U: " << U << std::endl;
    //std::cout << "W: " << W << std::endl;
    //std::cout << "Vt: " << Vt << std::endl;
    //cv::Mat Martix_V = Vt.row(8).t();
    //cv::Mat Martix_H = cv::Mat::zeros(3, 3, CV_32FC1);
    //for (int i = 0; i < 3; i++) {
    //    for (int j = 0; j < 3; j++) {
    //        Martix_H.at<float>(i, j) = Martix_V.at<float>(i * 3 + j, 0);
    //    }
    //}

    cv::Mat Martix_H = Vt.row(8).reshape(0, 3);
    return Martix_H;
}

//! 通过重投影 测试单应性矩阵，并返回 误差均值
//! 参数4：_H_Reproject_Points : 各点重投影结果集合
//! 参数5：_H_Reproject_Errors : 各点重投影误差集合
//! 参数6：Normal_Flag : 单应性矩阵是否归一化
double _Reproject(
    std::vector<cv::Point3f> Object,
    std::vector<cv::Point2f> Image,
    cv::Mat _H,
    std::vector<cv::Point2f>& _H_Reproject_Points,
    std::vector<cv::Point2f>& _H_Reproject_Errors,
    bool Normal_Flag) {
    //! 单个点重投影结果，含尺度因子
    cv::Mat _Test_Reproject_Point = cv::Mat::zeros(3, 1, CV_32FC1);
    //! 单个点重投影结果，去除尺度因子影响
    cv::Mat _Test_Res;

    for (int i = 0; i < Object.size(); i++) {
        _Test_Reproject_Point.at<float>(0, 0) = Object[i].x;
        _Test_Reproject_Point.at<float>(1, 0) = Object[i].y;
        _Test_Reproject_Point.at<float>(2, 0) = 1.0;

        //! 单应性矩阵与测试点（P1）相乘 = 重投影点
        _Test_Res = _H * _Test_Reproject_Point;
        if (Normal_Flag) {
            //! 重投影后各点位置
            _H_Reproject_Points.push_back({
                _Test_Res.at<float>(0, 0) / _Test_Res.at<float>(2, 0),
                _Test_Res.at<float>(1, 0) / _Test_Res.at<float>(2, 0)
                });
            //! 重投影后各点误差
            _H_Reproject_Errors.push_back({
                fabs(_Test_Res.at<float>(0, 0) / _Test_Res.at<float>(2, 0) - Image[i].x),
                fabs(_Test_Res.at<float>(1, 0) / _Test_Res.at<float>(2, 0) - Image[i].y)
                });
        }
        else {
            //! 重投影后各点位置
            _H_Reproject_Points.push_back({
                _Test_Res.at<float>(0, 0) ,
                _Test_Res.at<float>(1, 0)
                });
            //! 重投影后各点误差
            _H_Reproject_Errors.push_back({
                fabs(_Test_Res.at<float>(0, 0) - Image[i].x),
                fabs(_Test_Res.at<float>(1, 0) - Image[i].y)
                });
        }
    }
    //! 均值误差
    double _H_Reproject_Error_Mean = cv::mean(_H_Reproject_Errors)[0];
    return _H_Reproject_Error_Mean;
}
