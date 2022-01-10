//
// Created by Dingyi Gu on 2021/10/26.
//

#include "calibrate_eih.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

/**
 * @author dingyi
 * @start_date 10/26/2021
 * @modified_date 11/10/2021
 * @calibrate_handeye
 */



// Declare calculateETH constructor
calculateEIH::calculateEIH() = default;

/**
 * Read dataset from robot
 * @param fileName
 * @param matrixOfPose
 * @return
 */
// Declare calculateEIH member function
// Read data from file in pose
bool calculateEIH::getFileContent(std::string  fileName, int dataPtsNum, dataset &matrixOfPose){
    int sizeOfPose = 6; //fixed
    int sizeOfTargets = dataPtsNum;  //Modify it according to the number of targets
    matrixOfPose.resize(sizeOfTargets,sizeOfPose);
    // Open the file
    std::ifstream in(fileName);
    // Check if object is valid
    if(!in){
        std::cerr << "Cannot open the file: " << fileName << std::endl;
        return false;
    }

    for (int i = 0; i <= sizeOfTargets-1; i++  ){
        for (int j = 0; j<= sizeOfPose-1; j++){
            in >> matrixOfPose(i,j);
        }
    }
    // Close the file
    in.close();
    return true;
}


/**
 * Read dataset from camera
 * @param fileName2
 * @param matrixOfhT
 * @return
 */
// Read data from file in transformation matrix
bool calculateEIH::getFileContent2(std::string fileName2, int dataPtsNum2, dataset &matrixOfhT){
    int sizeOfCol = 4;
    int sizeOfRow = 4*((dataPtsNum2)-1); //Modify it according to the number of targets
    matrixOfhT.resize(sizeOfRow,sizeOfCol);
    // Open the file
    std::ifstream in(fileName2);
    // Check if object is valid
    if(!in){
        std::cerr << "Cannot open the file: " << fileName2 << std::endl;
        return false;
    }

    for (int i = 0; i <= sizeOfRow-1; i++  ){
        for (int j = 0; j<= sizeOfCol-1; j++){
            in >> matrixOfhT(i,j);
        }
    }
    // Close the file
    in.close();
    return true;


}



/**
 * rpy2tr
 * @param matrixOfPose
 * @param hT_b2f
 * @return
 */
bool calculateEIH::getTransFromRPY(dataset matrixOfPose, std::vector<hT4d> &hT_b2f){

    int sizeOfPose = matrixOfPose.cols();
    int sizeOfTargets = matrixOfPose.rows();
    eul3d roll_vec; // roll container
    eul3d pitch_vec; // pitch container
    eul3d yaw_vec; // yaw container
    std::vector<transl3d> translVec;
    std::vector<rot3d> rotMat;
    std::vector<quaTmp> quaternion;




    for (int i = 0; i <= sizeOfTargets-1; i++ ){
        double rollTmp = matrixOfPose(i,sizeOfPose-3)*(M_PI/180); //deg2rad
        double pitchTmp = matrixOfPose(i,sizeOfPose-2)*(M_PI/180); //deg2rad
        double yawTmp = matrixOfPose(i,sizeOfPose-1)*(M_PI/180); //deg2rad
        transl3d translTmp = matrixOfPose.block(i,0,1,3).transpose();
        Eigen::Quaterniond quaTmp =  Eigen::AngleAxisd(yawTmp, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(pitchTmp, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(rollTmp, Eigen::Vector3d::UnitX());
        rot3d rotTmp = quaTmp.toRotationMatrix();

        hT4d hT_b2f_tmp = Eigen::Isometry3d::Identity(); //initialization
        hT_b2f_tmp.rotate(rotTmp);
        hT_b2f_tmp.pretranslate(translTmp);

        translVec.push_back(translTmp);
        rotMat.push_back(rotTmp);
        quaternion.push_back(quaTmp);
        hT_b2f.push_back(hT_b2f_tmp);
    }

    return true;
}


/**
 * rv2tr
 * @param matrixOfPose
 * @param hT_b2f
 * @return
 */
bool calculateEIH::getTransFromRotVec(dataset matrixOfPose, std::vector<hT4d> &hT_b2f) {
    int sizeOfPose = matrixOfPose.cols();
    int sizeOfTargets = matrixOfPose.rows();
    std::vector<transl3d> translVec;
    std::vector<rot3d> rotMat;
    std::vector<vec3d> rotVec;

    for (int i = 0; i <= sizeOfTargets-1; i++ ){
        double Rx_tmp = matrixOfPose(i,sizeOfPose-3); //* (M_PI/180); //deg2rad
        double Ry_tmp = matrixOfPose(i,sizeOfPose-2); //* (M_PI/180); //deg2rad
        double Rz_tmp = matrixOfPose(i,sizeOfPose-1); //* (M_PI/180); //deg2rad
        vec3d rotVec_tmp;
        rotVec_tmp(0) = Rx_tmp;
        rotVec_tmp(1) = Ry_tmp;
        rotVec_tmp(2) = Rz_tmp;

        transl3d translTmp = matrixOfPose.block(i,0,1,3).transpose();
        translTmp = translTmp*1000; //m to mm

        double theta = rotVec_tmp.norm(); // theta = sqrt(rx^2 + ry^2 + rz^2);

//        double cth = cos(theta);
//        double sth = sin(theta);
//        double vth = 1 - cos(theta);
        rotVec_tmp.normalize(); // [kx ky kz];
        /*
        double kx = rotVec_tmp(0);
        double ky = rotVec_tmp(1);
        double kz = rotVec_tmp(2);
        rot3d rotMat_tmp;
        rotMat_tmp(0,0) = kx*kx*vth+cth;
        rotMat_tmp(0,1) = kx*ky*vth-kz*sth;
        rotMat_tmp(0,2) = kx*kz*vth+ky*sth;
        rotMat_tmp(1,0) = kx*ky*vth+kz*sth;
        rotMat_tmp(1,1) = ky*ky*vth+cth;
        rotMat_tmp(1,2) = ky*kz*vth-kx*sth;
        rotMat_tmp(2,0) = kx*kz*vth-ky*sth;
        rotMat_tmp(2,1) = ky*kz*vth+kx*sth;
        rotMat_tmp(2,2) = kz*kz*vth+cth;

        double beta;
        beta = atan2(-rotMat_tmp(2,0),sqrt(rotMat_tmp(0,0)*rotMat_tmp(0,0)+rotMat_tmp(1,0)*rotMat_tmp(1,0)));
        std::cout << beta <<"\n";
        */

        Eigen::AngleAxisd rotMat_tmp(theta, rotVec_tmp);

        hT4d hT_b2f_tmp = Eigen::Isometry3d::Identity(); //initialization
        hT_b2f_tmp.rotate(rotMat_tmp);
        hT_b2f_tmp.pretranslate(translTmp);

        translVec.push_back(translTmp);
        rotVec.push_back(rotVec_tmp);
        hT_b2f.push_back(hT_b2f_tmp);
    }

    return true;
}


void calculateEIH::CalculateT_A(std::vector<hT4d> hT_b2f, std::string fileName, std::vector<mat4d> &T_L) {
    mat4d T_L_tmp;
    int sizeOfTargets = hT_b2f.size();
    std::ofstream out(fileName);
    std::cout << sizeOfTargets << "\n";
    for(int i = 0; i <= (sizeOfTargets-1)-1; i++){
        T_L_tmp = hT_b2f.at(i).matrix().inverse() * hT_b2f.at(i+1).matrix();
//        out << "Trans [" << i <<"] to [" << i+1 << "] is: " << "\n";
        out << T_L_tmp.matrix() << std::endl;
        T_L.push_back(T_L_tmp);
    }
    out.close();
}





// Park and Martin's Method Starts Here ===================================================================================

/**
 * Decompose A and B to R_A, R_B, t_A and t_B
 * @param hT_b2f
 * @param T_R
 * @param R_A
 * @param R_B
 * @param t_A
 * @param t_B
 * @return
 */
bool calculateEIH::DecomposeMatAB(std::vector<mat4d> T_L, std::vector<mat4d> T_R, std::vector<rot3d> &R_A,
                                  std::vector<rot3d> &R_B, std::vector<transl3d> &t_A, std::vector<transl3d> &t_B) {
    int sizeOfTargets = T_R.size()+1;
    mat4d T_L_tmp, T_R_tmp;
    rot3d R_A_tmp, R_B_tmp;
    transl3d t_A_tmp, t_B_tmp;

    for(int i = 0; i <= (sizeOfTargets-1)-1; i++){
        T_L_tmp = T_L.at(i);
        T_R_tmp = T_R.at(i);

        R_A_tmp = T_L_tmp.block(0,0,3,3);
        t_A_tmp = T_L_tmp.block(0,3,3,1);
        R_B_tmp = T_R_tmp.block(0,0,3,3);
        t_B_tmp = T_R_tmp.block(0,3,3,1);

        T_L.push_back(T_L_tmp);
        R_A.push_back(R_A_tmp);
        t_A.push_back(t_A_tmp);
        R_B.push_back(R_B_tmp);
        t_B.push_back(t_B_tmp);
    }


    return true;
}



/**
 * Calculate R_x
 * @param R_A
 * @param R_B
 * @param R_x
 * @return
 */
bool calculateEIH::calibrateEIH_Rx(std::vector<rot3d> R_A, std::vector<rot3d> R_B, rot3d &R_x) {
    // Polar Decomposition
    std::vector<rot3d> alpha, beta;
    rot3d alpha_tmp, beta_tmp;
    int sizeOfTargets;
    rot3d M;


    sizeOfTargets = R_A.size();
    //sizeOfTargets = 15; //Modify it according to the number of targets
    for(int j = 0; j <= sizeOfTargets-1; j++){
        alpha_tmp = R_A.at(j).log();
        beta_tmp = R_B.at(j).log();

        alpha.push_back(alpha_tmp);
        beta.push_back(beta_tmp);
    }

    M.setZero();

    std::cout << "\n";
    std::cout << "Size of Targets is: " << "\n";
    std::cout << sizeOfTargets << std::endl;

    for(int i = 0; i <= sizeOfTargets-1; i++){
        M = M + beta.at(i)*(alpha.at(i).transpose());
    }

    rot3d M_tmp = (M.transpose()*M).inverse();
    R_x = M_tmp.sqrt()*M.transpose();

    return true;
}


/**
 * Calculate t_x
 * @param R_A
 * @param t_A
 * @param t_B
 * @param R_x
 * @param t_x
 * @return
 */
bool calculateEIH::calibrateEIH_Tx(std::vector<rot3d> R_A, std::vector<transl3d> t_A,
                                   std::vector<transl3d> t_B, rot3d R_x, transl3d &t_x) {
    int sizeOfTargets;
    Eigen::MatrixXd A_t, B_t;


    sizeOfTargets = t_A.size();
    //sizeOfTargets = 15; //Modify it according to the number of targets
    A_t.resize(3*sizeOfTargets,3);
    B_t.resize(3*sizeOfTargets,1);
    A_t.setZero();
    B_t.setZero();
    Eigen::Matrix3d matI3d = Eigen::Matrix3d::Identity();

    for (int i = 0; i <= sizeOfTargets-1; i++){
        A_t.block(3*i,0,3,3) = matI3d - R_A.at(i);
        B_t.block(3*i,0,3,1) = t_A.at(i) - R_x*t_B.at(i);
    }

    t_x = A_t.completeOrthogonalDecomposition().pseudoInverse()*B_t;
    return true;
}


/**
 * Calculate mean rotation error
 * @param R_x
 * @param R_A
 * @param R_B
 * @param R_err
 * @return
 */
bool calculateEIH::HandEyeError_Rot(rot3d R_x, std::vector<rot3d> R_A, std::vector<rot3d> R_B, double &R_err) {
    int sizeOfTargets;
    std::vector<rot3d> Rel_err;
    std::vector<vec3d> eul_err;
    std::vector<double> theta_err; //theta from rv
    rot3d Rel_err_tmp;
    vec3d eul_err_tmp;
    vec3d rv_err_tmp;
    double theta_err_tmp; //theta from rv

    sizeOfTargets = R_A.size();

    for(int i = 0; i <= sizeOfTargets-1; i++){
        Rel_err_tmp = (R_x * R_B.at(i)).inverse() * (R_A.at(i) * R_x);
        rv_err_tmp = rm2Rv(Rel_err_tmp); // convert rotation matrix to rotation vector
        theta_err_tmp = rv_err_tmp.norm();

        Rel_err.push_back(Rel_err_tmp);
        theta_err.push_back(theta_err_tmp);
    }


    double sum, R_err_tmp;
    double theta_tmp;
    sum = 0;

    for(int j = 0; j <= sizeOfTargets-1; j++){
        theta_tmp = theta_err.at(j);
        R_err_tmp = pow(theta_tmp,2);
        sum = sum + sqrt(R_err_tmp);
    }

    R_err = (sum/sizeOfTargets)*(180/M_PI);


    return true;
}



/**
 * Calculate mean translation error
 * @param R_x
 * @param t_x
 * @param R_A
 * @param t_A
 * @param t_B
 * @param t_err
 * @return
 */
bool calculateEIH::HandEyeError_transl(rot3d R_x, transl3d t_x, std::vector<rot3d> R_A, std::vector<transl3d> t_A,
                                       std::vector<transl3d> t_B, double &t_err) {
    int sizeOfTargets;
    vec3d Rel_err_tmp, Rel_err_ttmp;
    std::vector<vec3d> Rel_err;

    double t_err_tmp, sum;


    sizeOfTargets = R_A.size();
    std::cout << "\n";

    for(int i = 0; i <= sizeOfTargets-1; i++){
        Rel_err_tmp = (R_A.at(i) * t_x) + t_A.at(i) - (R_x * t_B.at(i) + t_x);
        //std::cout << Rel_err_tmp << "\n";
        //std::cout << "\n";
        //Rel_err_tmp.transposeInPlace();
        Rel_err.push_back(Rel_err_tmp);
    }

    sum = 0;

    for(int j = 0; j <= sizeOfTargets-1; j++){
        Rel_err_ttmp = Rel_err.at(j);
//        std::cout << Rel_err_ttmp << "\n";
        t_err_tmp = pow(Rel_err_ttmp(0),2) + pow(Rel_err_ttmp(1),2) + pow(Rel_err_ttmp(2),2);
        sum = sum + sqrt(t_err_tmp);
    }
    t_err = sum/sizeOfTargets;



    return true;
}



/**
 * Write hT to txt.file
 * @param v_X
 * @param fileName
 * @return
 */
bool calculateEIH::writeMatrix(hT4d v_X, std::string fileName) {
    std::ofstream out(fileName);
    out << v_X.matrix() << std::endl;
    out.close();

    return true;
}


/**
 * Write error metrics
 * @param errMetrics
 * @param fileName
 * @return
 */
bool calculateEIH::writeError(std::vector<double> errMetrics, std::string fileName) {
    std::ofstream out(fileName);
    out << "Mean rotation error(in deg) is: " << errMetrics.at(0) << "\n";
    out << "Mean translation error (in mm) is: " << errMetrics.at(1) << "\n";
    out.close();

    return true;
}

// Park and Martin's Method Ends Here ===================================================================================






// Tsai Lenz's Method Starts Here =======================================================================================

bool calculateEIH::getAB_Tsai(std::vector<mat4d> T_R, std::vector<mat4d> T_L,
                                    vAvB &v_A, vAvB &v_B) {
    v_A.resize(4,8);
    v_B.resize(4,8);

    mat4d T_L1 = T_L.at(0);
    mat4d T_L2 = T_L.at(1);
    v_A.topLeftCorner<4,4>() = T_L1;
    v_A.topRightCorner<4,4>() = T_L2;

    mat4d T_R1 = T_R.at(0);
    mat4d T_R2 = T_R.at(1);
    v_B.topLeftCorner<4,4>() = T_R1;
    v_B.topRightCorner<4,4>() = T_R2;

    return true;
}



bool calculateEIH::implementTsai(vAvB vA, vAvB vB, hT4d &vX) {
    vAvB vA1, vB1;
    rot3d Rcg;
    vec3d va, vb, vx, Tcg;


    vA.resize(4, 8);
    vB.resize(4, 8);
    vA1.resize(3, 3);
    vB1.resize(3, 3);

    int n = vA.cols();
    double theta;

    n = n / 4;

    matXd matS, matV, matC, matD;
    matS.resize(3 * n, 3);
    matS.setZero();
    matV.resize(3 * n, 1);
    matV.setZero();
    matC.resize(3 * n, 3);
    matC.setZero();
    matD.resize(3 * n, 1);
    matD.setZero();

    // Calculate Rcg
    for (int i = 0; i <= n - 1; i++) {
        vA1 = vA.block(0, 4 * i, 3, 3);
        vB1 = vB.block(0, 4 * i, 3, 3);
        vA1 = vA1.log();
        vB1 = vB1.log();

        va(0) = vA1(2, 1);
        va(1) = vA1(0, 2);
        va(2) = vA1(1, 0);
        vb(0) = vB1(2, 1);
        vb(1) = vB1(0, 2);
        vb(2) = vB1(1, 0);
        va.normalize();
        vb.normalize();

        /*
        std::cout << "vA1 is: " << "\n";
        std::cout << vA1 << std::endl;
        std::cout <<"\n";

        std::cout << "vB1 is: " << "\n";
        std::cout << vB1 << std::endl;
        std::cout <<"\n";

        std::cout << "va is: " << "\n";
        std::cout << va << std::endl;
        std::cout <<"\n";

        std::cout << "vb is: " << "\n";
        std::cout << vb << std::endl;
        std::cout <<"\n";
        */

        /*
        skew3d skew_tmp = getVec2Skew(va + vb);
        vec3d vaMb_tmp = va - vb;
        for (int k = 0; k <= 2; k++) {
            matV(k + 3 * i, 0) = vaMb_tmp(k);
            for (int j = 0; j <= 2; j++) {
                matS(k + 3 * i, j) = skew_tmp(k, j);
            }
        }
        */

        matS.block(3*i,0,3,3) = vec2Skew(va+vb);
        matV.block(3*i,0,3,1) = va-vb;
    }

    /*
    std::cout << "matS is: " << "\n";
    std::cout << matS << std::endl;
    std::cout <<"\n";

    std::cout << "matV is: " << "\n";
    std::cout << matV << std::endl;
    std::cout <<"\n";
    */


    double coe1 = 2.0, coe2 = 1.0;

    vx = matS.completeOrthogonalDecomposition().pseudoInverse() * matV;

    theta = coe1*atan(vx.norm());

    vx.normalize();

    Rcg = (Eigen::Matrix3d::Identity() * cos(theta) + sin(theta) * vec2Skew(vx) + (coe2 - cos(theta)) * vx * vx.transpose());

    Rcg.transposeInPlace();
    std::cout << "Rcg is: " << "\n";
    std::cout << Rcg << std::endl;
    std::cout <<"\n";

    // Calculate Tcg
    Eigen::Matrix3d matI = Eigen::Matrix3d::Identity();

    for(int j = 0; j <= n-1; j++){
        matC.block(3*j,0,3,3) = matI - vA.block(0,4*j,3,3);
        matD.block(3*j,0,3,1) = vA.block(0,4*j+3,3,1) - Rcg*vB.block(0,4*j+3,3,1);
    }
    /*
    std::cout << "matC is: " << "\n";
    std::cout << matC << std::endl;
    std::cout <<"\n";

    std::cout << "matD is: " << "\n";
    std::cout << matD << std::endl;
    std::cout <<"\n";
    */

    Tcg = matC.completeOrthogonalDecomposition().pseudoInverse()*matD;
    std::cout << "Tcg is: " << "\n";
    std::cout << Tcg << std::endl;
    std::cout <<"\n";


    vX = Eigen::Isometry3d::Identity();
    vX.rotate(Rcg);
    vX.pretranslate(Tcg);

    return true;
}


// Tsai Lenz's Method Ends Here =========================================================================================


//bool calculateEIH::getTransFromCamera(dataset matrixOfCamera, std::vector<mat4d> &hT_c2o) {
//    int sizeOfCol = matrixOfCamera.cols();
//    int sizeOfRow = matrixOfCamera.rows();
//
//    for(int i = 0; i <= sizeOfRow-1; i=i+4){
//        mat4d hTTmp = matrixOfCamera.block(i,0,4,4);
//        hT_c2o.push_back(hTTmp);
//    }
//
//    return true;
//}








/**
 * Convert vector to skew matrix
 * @param vecX
 * @return skew
 */
skew3d calculateEIH::vec2Skew(vec3d vecX) {
    skew3d skew;
    skew.setZero(); //skew(0,0) = 0;
    skew(0,1) = -vecX(2);
    skew(0,2) = vecX(1);
    skew(1,0) = vecX(2);
//  skew(1,1) = 0;
    skew(1,2) = -vecX(0);
    skew(2,0) = -vecX(1);
    skew(2,1) = vecX(0);
//  skew(2,2) = 0;

    return skew;
}


/**
 * Convert rotation matrix to rotation vector (Rodrigues' Formula)
 * @param R
 * @return rv
 */
vec3d calculateEIH::rm2Rv(rot3d R) {
    double theta, r11, r12, r13, r21, r22, r23, r31, r32, r33;
    double sth, kx, ky, kz, rx, ry, rz;

    r11 = R(0,0);
    r12 = R(0,1);
    r13 = R(0,2);
    r21 = R(1,0);
    r22 = R(1,1);
    r23 = R(1,2);
    r31 = R(2,0);
    r32 = R(2,1);
    r33 = R(2,2);

    theta = std::acos((r11+r22+r33-1)/2);
    sth = sin(theta);
    kx = (r32-r23)/(2*sth);
    ky = (r13-r31)/(2*sth);
    kz = (r21-r12)/(2*sth);
    rx = theta*kx;
    ry = theta*ky;
    rz = theta*kz;

    vec3d rv;
    rv << rx, ry, rz;

    return rv;
}




