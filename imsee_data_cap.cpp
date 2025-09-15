#include <iostream>
#include <unistd.h>
#include <string.h>
#include "DriverInterface.h"

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

struct CameraCalibrationParameter
{
  int _width;   // 图像宽
  int _height;  // 图像高
  int _channel; // 通道数

  double _Kl[9];                // 3X3 左相机内参矩阵
  double _Kr[9];                // 3X3 右相机内参矩阵
  double _Dl[4];                // 4X1 左相机畸变差校正参数,鱼眼畸变
  double _Dr[4];                // 4X1 右相机畸变差校正参数,鱼眼畸变
  double _Pl[12];               // 3X4 基线校正后左相机投影矩阵
  double _Pr[12];               // 3X4 基线校正后右相机投影矩阵
  double _Rl[9];                // 3X3 基线校正后左相机旋转矩阵
  double _Rr[9];                // 3X3 基线校正后右相机旋转矩阵
  double _TSCl[16];             // 4X4 左相机系到传感器坐标系的变换
  double _TSCr[16];             // 4X4 右相机系到传感器坐标系的变换
  double _focal_length_l[2];    // 相机fx,fy
  double _focal_length_r[2];    // 相机fx,fy
  double _principal_point_l[2]; // 相机cx,cy
  double _principal_point_r[2]; // 相机cx,cy
  double _baseline;             // 基线，单位：m

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

struct CameraParameter
{
  double _TSC[16]; //
  /** 4X4 matrix from camera to imu */
  int _width;
  /** width */
  int _height;
  /** height */
  double _focal_length[2];
  /** fx,fy */
  double _principal_point[2];
  /** cx,cy */
  double _R[9];
  /** Rectification matrix (stereo cameras only)
     A rotation matrix aligning the camera coordinate system to the ideal
     stereo image plane so that epipolar lines in both stereo images are
     parallel. */

  double _P[12];
  /** Projection/camera matrix
         [fx'  0  cx' Tx]
     P = [ 0  fy' cy' Ty]
         [ 0   0   1   0] */
  double _K[9];
  /** Intrinsic camera matrix for the raw (distorted) images.
         [fx  0 cx]
     K = [ 0 fy cy]
         [ 0  0  1] */
  double _D[4];
  /** The distortion parameters, size depending on the distortion model.
     For us, the 4 parameters are: (k1, k2, t1, t2).*/
  CameraParameter resize(double ratio) const
  {
    CameraParameter parameter;
    parameter = *this;
    parameter._width = static_cast<int>(this->_width * ratio);
    parameter._height = static_cast<int>(this->_height * ratio);
    for (int m = 0; m < 2; m++)
    {
      parameter._focal_length[m] = ratio * this->_focal_length[m];
    }
    for (int n = 0; n < 2; n++)
    {
      parameter._principal_point[n] = ratio * this->_principal_point[n];
    }
    for (int i = 0; i < 12; i++)
    {
      if (i != 10)
        parameter._P[i] = ratio * this->_P[i];
    }
    for (int j = 0; j < 9; j++)
    {
      if (j != 8)
        parameter._K[j] = ratio * this->_K[j];
    }
    return parameter;
  }
  void printInfo()
  {
    std::cout << "_width: " << _width << ", height: " << _height << std::endl;
    for (int i = 0; i < 2; i++)
    {
      std::cout << "_focal_length[" << i << "]: " << _focal_length[i]
                << std::endl;
      std::cout << "_principal_point[" << i << "]: " << _principal_point[i]
                << std::endl;
    }
    std::cout << "_focal_length: { ";
    for (int i = 0; i < 2; i++)
    {
      std::cout << _focal_length[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_principal_point: { ";
    for (int i = 0; i < 2; i++)
    {
      std::cout << _principal_point[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_TSC: { ";
    for (int i = 0; i < 16; i++)
    {
      std::cout << _TSC[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_R: { ";
    for (int i = 0; i < 9; i++)
    {
      std::cout << _R[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_P: { ";
    for (int i = 0; i < 12; i++)
    {
      std::cout << _P[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_K: { ";
    for (int i = 0; i < 9; i++)
    {
      std::cout << _K[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_D: { ";
    for (int i = 0; i < 4; i++)
    {
      std::cout << _D[i] << " ";
    }
    std::cout << "} " << std::endl;
  }
};
// IMU 参数
struct IMUParameter
{
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
  void printInfo()
  {
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
    for (int i = 0; i < 4; i++)
    {
      std::cout << _a0[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_T_BS: { ";
    for (int i = 0; i < 16; i++)
    {
      std::cout << _T_BS[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_Acc: { ";
    for (int i = 0; i < 12; i++)
    {
      std::cout << _Acc[i] << " ";
    }
    std::cout << "} " << std::endl;
    std::cout << "_Gyr: { ";
    for (int i = 0; i < 12; i++)
    {
      std::cout << _Gyr[i] << " ";
    }
    std::cout << "} " << std::endl;
  }
};

struct ModuleInfo
{
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
  void printInfo()
  {
    std::cout << "id: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _id[i];
    }
    std::cout << std::endl
              << "designer: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _designer[i];
    }
    std::cout << std::endl
              << "fireware_version: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _fireware_version[i];
    }
    std::cout << std::endl
              << "hardware_version: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _hardware_version[i];
    }
    std::cout << std::endl
              << "lens: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _lens[i];
    }
    std::cout << std::endl
              << "imu: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _imu[i];
    }
    std::cout << std::endl
              << "viewing_angle: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _viewing_angle[i];
    }
    std::cout << std::endl
              << "_baseline: ";
    for (int i = 0; i < 32; i++)
    {
      std::cout << _baseline[i];
    }
    std::cout << std::endl;
  }
};

struct SlamParameter
{
  int _numKeyframes;
  /** 关键帧数量(默认为５) */
  int _numImuFrames;
  /** 相邻imu帧数量(默认为３) */
  int _ceres_minIterations;
  /** 最小优化次数(默认为３) */
  int _ceres_maxIterations;
  /** 最大优化次数(默认为４) */
  double _ceres_timeLimit;
  /** ceres　time　limit(默认为3.5000000000000003e-02) */
  int detection_threshold;
  /** 角点阈值(默认为30) */
  int detection_octaves;
  /** detection octaves(默认为0) */
  int detection_maxNoKeypoints;
  /** 最大角点数量(默认为100) */
  bool displayImages;
  /** 是否显示slam界面(默认为true) */
};

struct ModuleParameters
{
  CameraParameter _camera[2]; // 左右目相机
  /** CameraParameter of cameras */
  IMUParameter _imu;
  /** IMU parameter */
  ModuleInfo _device;
  /** device info */
  SlamParameter _slam;
  /** slam parameter */
};

void cam_callback(indem::cameraData *data)
{
#ifdef USE_OPENCV
  cv::Mat m = cv::Mat(cv::Size(data->_width, data->_height), CV_8UC1, (void *)data->_image);
  cv::Rect l(0, 0, data->_width / 2, data->_height);
  cv::Rect r(data->_width / 2, 0, data->_width / 2, data->_height);

  cv::imshow("left", m(l));
  cv::imshow("right", m(r));
  cv::waitKey(10);
#else
  std::cout<<"get img:"<<data->_timeStamp<< " w " << data->_width<< " h "<< data->_height<<" c "<< data->_channel<<" s "<< data->_size<<std::endl;
#endif

}

void img_callback(indem::IMUData *data)
{
  const double g = 9.802;
  std::cout << "get imu:" << data->_timeStamp << " acc " << data->_acc[0] * g << "," << data->_acc[1] * g << "," << data->_acc[2] * g << " gyr " << data->_gyr[0] << "," << data->_gyr[1] << "," << data->_gyr[2] << std::endl;
}

void hotplug_callback(bool bArrive)
{
}

int main(int argc, char **argv)
{
  std::cout << "start open device" << std::endl;
  SetHotplugCallback(hotplug_callback);

  indem::IDriverInterface *di = DriverFactory();
  di->SetCameraCallback(&cam_callback);
  di->SetIMUCallback(&img_callback);

  int version = 0;
  size_t info_size = 0;
  unsigned char module_info[2048];
  struct ModuleParameters moddule_param;
  di->GetModuleParams(version, module_info, info_size);
  memcpy(&moddule_param, module_info, sizeof(struct ModuleParameters));

  std::cout << "#####################" << std::endl;
  std::cout << "version:" << version << std::endl;
  std::cout << "info_size:" << info_size << std::endl;
  moddule_param._device.printInfo();
  moddule_param._imu.printInfo();
  moddule_param._camera[0].printInfo();
  moddule_param._camera[1].printInfo();
  std::cout << "#####################" << std::endl;

  bool sc = di->Open(1000, 50, indem::IMAGE_RESOLUTION::RESOLUTION_640); // imu hz,img hz
  std::cout << "open device " << (sc ? "successful" : "failed") << std::endl;

  while (sc)
  {
    usleep(1000);
  }

  di->Close();
}
