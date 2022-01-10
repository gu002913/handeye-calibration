//
// Created by Dingyi Gu on 2021/10/26.
//

#ifndef TEST_CALIBRATE_EIH_H
#define TEST_CALIBRATE_EIH_H

//#define _USE_MATH_DEFINES //the symbols may be defined somewhere else
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>



typedef Eigen::Vector3d	  transl3d;      //3by1(double) translational vector(xyz)
typedef Eigen::Vector3d   eul3d;         //3by1(double) euler angles vector(uvw)
typedef Eigen::Vector3d   vec3d;

typedef Eigen::Matrix3d	  rot3d;         //3by3(double) rotational matrix
typedef Eigen::Matrix3d   skew3d;        //Skew matrix
typedef Eigen::MatrixXd	  dataset;       //dataset read from txt file (dynamic)
typedef Eigen::MatrixXd   vAvB;          //AX = XB, where A and B are 4by8(double) matrix
typedef Eigen::MatrixXd   matXd;

typedef Eigen::Matrix4d   mat4d;
typedef Eigen::Isometry3d hT4d;          //4by4(double) homogeneous transformation matrix
typedef Eigen::Quaterniond quaTmp;        //Quaternion(double)

// calculate eye in hand

class calculateEIH{

public:
    calculateEIH();

    bool getFileContent(std::string fileName, int dataPtsNum, dataset &matrixOfPose); //Read dataset from pose in euler

    bool getFileContent2(std::string fileName2, int dataPtsNum2, dataset &matrixOfhT); //Read dataset from homogeneous transformation

    bool getTransFromRPY(dataset matrixOfPose, std::vector<hT4d> &hT_b2f); //Homogeneous from base to flange

    // bool getTransFromRotVec(dataset matrixOfPose, std::vector<hT4d> &hT_b2f);


    // bool getTransFromCamera(dataset matrixOfCamera, std::vector<mat4d> &hT_c2o); //Homogeneous from camera to object

    void CalculateT_A(std::vector<hT4d> hT_b2f, std::string fileName, std::vector<mat4d> &T_L); // T_L
    bool DecomposeMatAB(std::vector<mat4d> T_L, std::vector<mat4d> T_R, std::vector<rot3d> &R_A, std::vector<rot3d> &R_B,
                        std::vector<transl3d> &t_A, std::vector<transl3d> &t_B);

    skew3d vec2Skew(vec3d vecX);

    bool calibrateEIH_Rx(std::vector<rot3d> R_A, std::vector<rot3d> R_B, rot3d &R_x); //implementTsai to solve Rx from AX = XB with least square(more than 3 data points)

    bool calibrateEIH_Tx(std::vector<rot3d> R_A, std::vector<transl3d> t_A, std::vector<transl3d> t_B, rot3d R_x, transl3d &t_x ); //implementTsai to solve tx from AX = XB with least square(more than 3 data points)

    bool HandEyeError_Rot(rot3d R_x, std::vector<rot3d> R_A, std::vector<rot3d> R_B, double &R_err);

    bool HandEyeError_transl(rot3d R_x, transl3d t_x, std::vector<rot3d> R_A,
                             std::vector<transl3d> t_A, std::vector<transl3d> t_B, double &t_err);


    bool getAB_Tsai(std::vector<mat4d> T_R, std::vector<mat4d> T_L, vAvB &v_A, vAvB &v_B);
    bool implementTsai(vAvB vA, vAvB vB, hT4d &vX); //AX = XB, X = tsai(A,B) for 3 data points

    vec3d rm2Rv(rot3d R); //Rotation matrix to rotation vector;

    bool writeMatrix(hT4d v_X, std::string fileName);
    bool writeError(std::vector<double> errMetrics, std::string fileName);


private:






};




#endif //TEST_CALIBRATE_EIH_H
