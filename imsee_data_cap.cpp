#include <iostream>
#include <unistd.h>
#include "DriverInterface.h"

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

struct ModuleInfo {
  char _id[32];
  /** id */
  char _designer[32];
  /** designer */
  char _fireware_version[32];
  /** fireware version */
  char _hardware_version[32];
  /** _hardware version */
  char _lens[32];
  /** lens */
  char _imu[32];
  /** imu */
  char _viewing_angle[32];
  /** viewing angle */
  char _baseline[32];
  /** baseline */
  void printInfo() {
    std::cout << "id: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _id[i];
    }
    std::cout << std::endl << "designer: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _designer[i];
    }
    std::cout << std::endl << "fireware_version: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _fireware_version[i];
    }
    std::cout << std::endl << "hardware_version: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _hardware_version[i];
    }
    std::cout << std::endl << "lens: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _lens[i];
    }
    std::cout << std::endl << "imu: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _imu[i];
    }
    std::cout << std::endl << "viewing_angle: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _viewing_angle[i];
    }
    std::cout << std::endl << "_baseline: ";
    for (int i = 0; i < 32; i++) {
      std::cout << _baseline[i];
    }
    std::cout << std::endl;
  }
};
// IMU 参数
struct IMUParameter {
  double _a_max;
  /** a_max */
  double _g_max;
  /** g_max */
  double _sigma_g_c;
  /** sigma_g_c */
  double _sigma_a_c;
  /** sigma_a_c */
  double _sigma_bg;
  /** sigma_bg */
  double _sigma_ba;
  /** sigma_ba */
  double _sigma_gw_c;
  /** sigma_gw_c */
  double _sigma_aw_c;
  /** sigma_aw_c */
  double _tau;
  /** tau */
  double _g;
  /** g */
  double _a0[4];
  /** a0 */
  double _T_BS[16];
  /** T_BS */
  double _Acc[12];
  /** Compensation parameters of accelerometer */
  double _Gyr[12];
  /** Compensation parameters of gyroscope */
  void printInfo() {
    std::cout << "_a_max: " << _a_max << std::endl
              << "_g_max: " << _g_max << std::endl
              << "_sigma_g_c: " << _sigma_g_c << std::endl
              << "_sigma_a_c: " << _sigma_a_c << std::endl
              << "_sigma_bg: " << _sigma_bg << std::endl
              << "_sigma_ba: " << _sigma_ba << std::endl
              << "_sigma_gw_c: " << _sigma_gw_c << std::endl
              << "_sigma_aw_c: " << _sigma_aw_c << std::endl
              << "_tau: " << _tau << std::endl
              << "_g: " << _g << std::endl;
    std::cout << "_a0: { ";
    for (int i = 0; i < 4; i++) {
      std::cout << _a0[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_T_BS: { ";
    for (int i = 0; i < 16; i++) {
      std::cout << _T_BS[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_Acc: { ";
    for (int i = 0; i < 12; i++) {
      std::cout << _Acc[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_Gyr: { ";
    for (int i = 0; i < 12; i++) {
      std::cout << _Gyr[i] << " ";
    }
    std::cout << "} " << std::endl;
  }
};


struct CameraCalibrationParameter {
  int _width;   //图像宽
  int _height;  //图像高
  int _channel; //通道数

  double _Kl[9];             // 3X3 左相机内参矩阵
  double _Kr[9];             // 3X3 右相机内参矩阵
  double _Dl[4];             // 4X1 左相机畸变差校正参数,鱼眼畸变
  double _Dr[4];             // 4X1 右相机畸变差校正参数,鱼眼畸变
  double _Pl[12];            // 3X4 基线校正后左相机投影矩阵
  double _Pr[12];            // 3X4 基线校正后右相机投影矩阵
  double _Rl[9];             // 3X3 基线校正后左相机旋转矩阵
  double _Rr[9];             // 3X3 基线校正后右相机旋转矩阵
  double _TSCl[16];          // 4X4 左相机系到传感器坐标系的变换
  double _TSCr[16];          // 4X4 右相机系到传感器坐标系的变换
  double _focal_length_l[2]; //相机fx,fy
  double _focal_length_r[2]; //相机fx,fy
  double _principal_point_l[2]; //相机cx,cy
  double _principal_point_r[2]; //相机cx,cy
  double _baseline;             //基线，单位：m

  double _AMax;
  double _GMax;
  double _SigmaGC;
  double _SigmaAC;
  double _SigmaBg;
  double _SigmaBa;
  double _SigmaGwC;
  double _SigmaAwC;

  /*  加计参数,3X4矩阵,每个元素如下
   *    x   y   z
   *    11  12  13
   *    21  22  23
   *    31  32  33
   */
  double _Acc[12];
  /*  陀螺参数,3X4矩阵,每个元素如下
   *    x   y   z
   *    11  12  13
   *    21  22  23
   *    31  32  33
   */
  double _Gyr[12];
};

void cam_callback(indem::cameraData *data)
{
    std::cout<<"get img:"<<data->_timeStamp<< " w " << data->_width<< " h "<< data->_height<<" c "<< data->_channel<<" s "<< data->_size<<std::endl;

#ifdef USE_OPENCV
    cv::Mat m = cv::Mat(cv::Size(data->_width,data->_height),CV_8UC1,(void*)data->_image);
    cv::Rect l(0,0,data->_width/2,data->_height);
    cv::Rect r(data->_width/2,0,data->_width/2,data->_height);
    

    cv::imshow("left", m(l));
    cv::imshow("right", m(r));
    cv::waitKey(10);
#endif
}

void img_callback(indem::IMUData *data)
{
    const double g = 9.802;
    std::cout<<"get imu:"<<data->_timeStamp<< " acc " << data->_acc[0]* g<< ","<< data->_acc[1]* g<<","<< data->_acc[2]* g<<" gyr " << data->_gyr[0]<< ","<< data->_gyr[1]<< ","<<data->_gyr[2]<<std::endl;
}


void hotplug_callback(bool bArrive)
{
}

int main(int argc,char** argv)
{
   std::cout<<"start open device"<<std::endl;
   SetHotplugCallback(hotplug_callback);

   indem::IDriverInterface * di = DriverFactory();
   di->SetCameraCallback(&cam_callback);
   di->SetIMUCallback(&img_callback);

   int version = -1;
   unsigned char params[2048];
   size_t len = 2048;
   di->GetModuleParams(version,params,len);

   std::cout<<"version:"<<version<<std::endl;
   std::cout<<"len:"<<len<<std::endl;
   
   bool sc = di->Open(1000,50,indem::IMAGE_RESOLUTION::RESOLUTION_640);//imu hz,img hz
   std::cout<<"open device "<<(sc?"successful":"failed")<<std::endl;
   
   while(sc)
   {
     usleep(1000);
   }

   di->Close();
}
