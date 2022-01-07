## **Eigen C++ Hand Eye Calibration**

***

This repository contains some sources to calibrate the homogeneous transformation from robot flange to a stereo pair of cameras, which is also known as eye in hand.

### **Dependencies**

* Eigen

```
% brew install eigen
```

### **Compilation**

Compile all the files using the following commands.

```
% mkdir cmake-build-debug
% cd cmake-build-debug
% cmake ..
% make
```

Make sure you are in the `cmake-build-debug` folder to run the exeutables.

```
% ./handeye-calibration
```

### **Data Analysis(calib_dataset)**

| File name                      | Description                                                                                              |
| ------------------------------ | -------------------------------------------------------------------------------------------------------- |
| base2flange_rpy.txt            | Flange pose recorded w.r.t robot base coordinate in roll pitch yaw (mm and deg).                         |
| T_B_hT.txt                     | Relative transformation of checkerboard from previous pose to current pose( w.r.t camera coordinate).    |
| <mark>T_A_hT.txt</mark>        | Relative transformation of robot flange from previous pose to current pose( w.r.t robot base coordinate. |
| <mark>flange2cam.txt</mark>    | Homogeneous transformation from robot flange to camera.                                                  |
| <mark>error_metrics.txt</mark> | Error analysis both in translation and rotation.                                                         |

<mark>Hightlighted .txt</mark> file is generated after executables.

```
Type 0 to implement transformation from base2flange_rpy.txt to T_A_hT.txt
Type 1 to implement Tsai Lenz's Calibration
Type 2 to implement Park and Martin's Calibration 
```

****<u>When implementing 0 and 2</u>****,

```js
void pose_rpy2T_L(){

    calculateEIH calculateEih;

    std::string filepath;
    int dataPtsNum;

    dataPtsNum = 28;
    filepath = "../calib_dataset/";

    ...

}

void Calibrate_Park_Martin(){

    calculateEIH calculateEih;

    std::string filepath;
    int dataPtsNum2;

    dataPtsNum2 = 28;
    filepath = "../calib_dataset/";

    ...
}
```

`dataPtsNum` , `dataPtsNum2` and `filepath` should be modified before executables.

Make sure the numbers of *base2flange_rpy.txt* and *T_B_hT.txt* in calib_dataset match `dataPtsNum -1` and `dataPtsNum2 -1` ,respectively.

****<u>When implementing 1</u>****,

choose two adjacent matrices either in *T_A_hT.txt* or *T_B_hT.txt*. 



### **References**

***

[1] Park, F.; Martin, B. Robot sensor calibration: Solving AX=XB on the Euclidean group. IEEE Trans. Robot. Autom. 1994, 10, 717–721, doi: 10.1109/70.326576.


