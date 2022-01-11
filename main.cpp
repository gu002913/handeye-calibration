#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include "calibrate_eih.h"



/**
 * @author dingyi
 * @start_date 10/26/2021
 * @modified_date 01/10/2022
 * @calibrate_handeye
 */



void pose_rpy2T_L();
void Calibrate_Park_Martin();
void Calibrate_Tsai();

int main() {


    std::cout << "Type 0 to implement transformation from base2flange_rpy.txt to T_A_hT.txt" << "\n";
    std::cout << "Type 1 to implement Tsai Lenz's Calibration" << "\n";
    std::cout << "Type 2 to implement Park and Martin's Calibration" << "\n";

    char key;
    std::cin >> key;

    switch(key){
        case '0' :
            pose_rpy2T_L();
            break;
        case '1' :
            Calibrate_Tsai();
            break;
        case '2' :
   Calibrate_Park_Martin();
            break;
        default :
            std::cerr << "Invalid input!" << "\n";
    }


    return 0;
}


void pose_rpy2T_L(){

    calculateEIH calculateEih;


    std::string filepath;
    int dataPtsNum;

    dataPtsNum = 12; //should be modified.
    filepath = "../calib_dataset/";

    dataset matrixOfPose;
    // Get the contents of file
    bool result = calculateEih.getFileContent(filepath+"base2flange_rpy.txt", dataPtsNum, matrixOfPose);
    if(result){
        std::cout << "\n";
        std::cout << "Read the raw dataset from (robot) txt file: " << std::endl;
        std::cout << std:: fixed << std::setprecision(6) << matrixOfPose << std::endl;
    }

    // Convert the robot pose data to Homogeneous Transformation Matrix
    std::vector<hT4d> hT_b2f;
    bool result3 = calculateEih.getTransFromRPY(matrixOfPose, hT_b2f);
    //    bool result3 = calculateEih.getTransFromRotVec(matrixOfPose, hT_b2f);
    if(result3){
        for(int i3 = 0; i3 <= dataPtsNum-1; i3++){
            std::cout << "\n";
            std::cout <<"The Homogeneous Transformation (robot) pose [" << i3 << "] is: " << std::endl;
            std::cout << hT_b2f.at(i3).matrix() << std::endl;
        }

    }

    std::vector<mat4d> T_L;
    calculateEih.CalculateT_A(hT_b2f, filepath+"T_A_hT.txt", T_L);

}

void Calibrate_Park_Martin(){

    calculateEIH calculateEih;


    std::string filepath;
    int dataPtsNum2;

    dataPtsNum2 = 12; //should be modified.
    filepath = "../calib_dataset/";

    dataset matrixOfhT, matrixOfhT_robot;
    std::vector<mat4d> T_R;
    bool result2 = calculateEih.getFileContent2(filepath + "T_B_hT.txt", dataPtsNum2, matrixOfhT);
    if (result2) {
        std::cout << "\n";
        std::cout << "Read the raw dataset from (camera) txt file: " << std::endl;
        std::cout << std::fixed << std::setprecision(6) << matrixOfhT << std::endl;

        mat4d T_Rtmp;
        for (int i2 = 0; i2 <= (dataPtsNum2 - 1) - 1; i2++) {
            T_Rtmp = matrixOfhT.block(4 * i2, 0, 4, 4);
            T_R.push_back(T_Rtmp);
            std::cout << "\n";
            std::cout << "TR[" << i2 << "," << i2 + 1 << "] is: " << std::endl;
            std::cout << T_R.at(i2) << std::endl;
        }
    }

    std::vector<mat4d> T_L;
    bool result2_1 = calculateEih.getFileContent2(filepath + "T_A_hT.txt", dataPtsNum2, matrixOfhT_robot);
    if (result2_1) {
        std::cout << "\n";
        std::cout << "Read the raw dataset from (robot) txt file: " << std::endl;
        std::cout << std::fixed << std::setprecision(6) << matrixOfhT_robot << std::endl;

        mat4d T_L_tmp;
        for (int i3 = 0; i3 <= (dataPtsNum2 - 1) - 1; i3++) {
            T_L_tmp = matrixOfhT_robot.block(4 * i3, 0, 4, 4);
            T_L.push_back(T_L_tmp);
            std::cout << "\n";
            std::cout << "TL[" << i3 << "," << i3 + 1 << "] is: " << std::endl;
            std::cout << T_L.at(i3) << std::endl;
        }
    }


    // Calculate and Decompose MatA and MatB
    std::vector<rot3d> R_A, R_B;
    std::vector<transl3d> t_A, t_B;
    bool result4 = calculateEih.DecomposeMatAB(T_L, T_R, R_A, R_B, t_A, t_B);
    if(result4){
        std::cout << "\n";
        std::cout << "Decomposition Success!" << std::endl;

    }


    rot3d R_x;
    transl3d t_x;
    hT4d v_X;
    bool result5 = calculateEih.calibrateEIH_Rx(R_A, R_B, R_x);
    if(result5){
        std::cout << "\n";
        std::cout << "Rx is: " << "\n";
        std::cout << R_x << std::endl;
    }



    bool result6 = calculateEih.calibrateEIH_Tx(R_A, t_A, t_B, R_x, t_x);
    if(result6){
        std::cout << "\n";
        std::cout << "Tx is: " << "\n";
        std::cout << t_x << std::endl;
        v_X = Eigen::Isometry3d::Identity();
        v_X.rotate(R_x);
        v_X.pretranslate(t_x);
        std::cout << "\n";
        std::cout << "X is: " << "\n";
        std::cout << v_X.matrix() << std::endl;
    }


    double R_err; //Error in rotation
    bool result8 = calculateEih.HandEyeError_Rot(R_x, R_A, R_B, R_err);
    if(result8){
        std::cout << "\n";
        std::cout << "Error in rotation (in deg) is: " << "\n";
        std::cout << R_err << std::endl;
    }


    double t_err; //Error in translation
    bool result9 = calculateEih.HandEyeError_transl(R_x, t_x, R_A, t_A, t_B, t_err);
    if(result9){
        std::cout << "\n";
        std::cout << "Error in translation (in mm) is: " << "\n";
        std::cout << t_err << std::endl;
    }


    bool result10 = calculateEih.writeMatrix(v_X, filepath+"flange2cam.txt");
    if(result10){
        std::cout << "\n";
        std::cout << "Write successfully!" << "\n";
    }

    std::vector<double> errMetrics;
    errMetrics = {R_err, t_err};
    bool result11 = calculateEih.writeError(errMetrics, filepath+"error_metrics.txt");
    if(result11){
        std::cout << "\n";
        std::cout << "Write successfully!" << "\n";
    }

}


void Calibrate_Tsai(){

    calculateEIH calculateEih;


    std::string filepath;
    int dataPtsNum3;

    dataPtsNum3 = 3;
    filepath = "../calib_dataset/";

    dataset matrixOfhT, matrixOfhT_robot;
    std::vector<mat4d> T_R;
    bool result2 = calculateEih.getFileContent2(filepath + "T_B_hT.txt", dataPtsNum3, matrixOfhT);
    if (result2) {
        std::cout << "\n";
        std::cout << "Read the raw dataset from (camera) txt file: " << std::endl;
        std::cout << std::fixed << std::setprecision(6) << matrixOfhT << std::endl;

        mat4d T_Rtmp;
        for (int i2 = 0; i2 <= (dataPtsNum3 - 1) - 1; i2++) {
            T_Rtmp = matrixOfhT.block(4 * i2, 0, 4, 4);
            T_R.push_back(T_Rtmp);
            std::cout << "\n";
            std::cout << "TR[" << i2 << "," << i2 + 1 << "] is: " << std::endl;
            std::cout << T_R.at(i2) << std::endl;
        }
    }

    std::vector<mat4d> T_L;
    bool result2_1 = calculateEih.getFileContent2(filepath + "T_A_hT.txt", dataPtsNum3, matrixOfhT_robot);
    if (result2_1) {
        std::cout << "\n";
        std::cout << "Read the raw dataset from (robot) txt file: " << std::endl;
        std::cout << std::fixed << std::setprecision(6) << matrixOfhT_robot << std::endl;

        mat4d T_L_tmp;
        for (int i3 = 0; i3 <= (dataPtsNum3 - 1) - 1; i3++) {
            T_L_tmp = matrixOfhT_robot.block(4 * i3, 0, 4, 4);
            T_L.push_back(T_L_tmp);
            std::cout << "\n";
            std::cout << "TL[" << i3 << "," << i3 + 1 << "] is: " << std::endl;
            std::cout << T_L.at(i3) << std::endl;
        }
    }

    vAvB v_A, v_B;
    bool result3 = calculateEih.getAB_Tsai(T_R, T_L, v_A, v_B);
    if(result3){
        std::cout << "\n";
        std::cout << "calculateEih.getAB_Tsai() Success!" << "\n";
        std::cout << "vA is: " << "\n";
        std::cout << v_A << "\n";
        std::cout << "vB is: " << "\n";
        std::cout << v_B << "\n";
    }

    hT4d v_X;
    bool result4 = calculateEih.implementTsai(v_A, v_B, v_X);
    if(result4){
        std::cout << "\n";
        std::cout << "calculateEih.implementTsai() Success!" << "\n";
        std::cout << "vX is: " << "\n";
        std::cout << v_X.matrix() << "\n";
    }

}