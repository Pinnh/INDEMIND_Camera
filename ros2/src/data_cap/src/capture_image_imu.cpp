#include <glob.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>

#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "DriverInterface.h"
#include "safe_queue.h"

const double g = 9.802;

using namespace std;
using namespace std::chrono_literals;

static ThreadSafeQueue<std::shared_ptr<sensor_msgs::msg::Imu>> imu_queue(100);
static ThreadSafeQueue<std::shared_ptr<cv_bridge::CvImage>> cam0_queue(10);
static ThreadSafeQueue<std::shared_ptr<cv_bridge::CvImage>> cam1_queue(10);
static indem::IDriverInterface *di;

double d2r(double deg)
{
    return deg / 180.0 * M_PI;
}

float d2r(float deg)
{
    return deg / 180.0f * M_PI;
}

double r2d(double rad)
{
    return rad / M_PI * 180.0;
}

float r2d(float rad)
{
    return rad / M_PI * 180.0f;
}

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
    // std::cout << "get img:" << data->_timeStamp << " w " << data->_width << " h " << data->_height << " c " << data->_channel << " s " << data->_size << std::endl;
    cv::Mat m = cv::Mat(cv::Size(data->_width, data->_height), CV_8UC1, (void *)data->_image);
    cv::Rect l(0, 0, data->_width / 2, data->_height);
    cv::Rect r(data->_width / 2, 0, data->_width / 2, data->_height);

    cv::Mat left_image = m(l);
    cv::Mat right_image = m(r);

    std::shared_ptr<cv_bridge::CvImage> msg0(new cv_bridge::CvImage());
    std::shared_ptr<cv_bridge::CvImage> msg1(new cv_bridge::CvImage());
    // cv_bridge::CvImage msg0, msg1;

    // msg0.header.stamp = this->get_clock()->now();
    double image_timestamp = data->_timeStamp;
    rclcpp::Time custom_time(image_timestamp * 1e6); // seconds, ns or ns
    msg0->header.stamp = custom_time;
    msg0->header.frame_id = "cam0";
    msg0->encoding = "mono8"; //"bgr8";
    msg0->image = left_image;

    msg1->header.stamp = custom_time;
    msg1->header.frame_id = "cam1";
    msg1->encoding = "mono8"; //"bgr8";
    msg1->image = right_image;

    // std::cout << "image_timestamp " << image_timestamp << std::endl;
    cam0_queue.Push(msg0);
    cam1_queue.Push(msg1);
    // cam0_image_pub->publish(*msg0.toImageMsg());
    // cam1_image_pub->publish(*msg1.toImageMsg());
    //  cv::imshow("left", m(l));
    //  cv::imshow("right", m(r));
    //  cv::waitKey(10);
}

void imu_callback(indem::IMUData *data)
{
    std::cout << "get imu:" << data->_timeStamp << " acc " << data->_acc[0] * g << "," << data->_acc[1] * g << "," << data->_acc[2] * g << " gyr " << data->_gyr[0] << "," << data->_gyr[1] << "," << data->_gyr[2] << std::endl;

    double imu_timestamp = data->_timeStamp;
    double acc_x = data->_acc[0] * g;
    double acc_y = data->_acc[1] * g;
    double acc_z = data->_acc[2] * g;

    double gyro_x = d2r(data->_gyr[0]);
    double gyro_y = d2r(data->_gyr[1]);
    double gyro_z = d2r(data->_gyr[2]);

    // sensor_msgs::msg::Imu imu_data;
    std::shared_ptr<sensor_msgs::msg::Imu> imu_data(new sensor_msgs::msg::Imu());
    imu_data->header.stamp = rclcpp::Time(imu_timestamp * 1e6);
    imu_data->header.frame_id = "imu0";

    // acc
    imu_data->linear_acceleration.x = acc_x;
    imu_data->linear_acceleration.y = acc_y;
    imu_data->linear_acceleration.z = acc_z;

    // gyro
    imu_data->angular_velocity.x = gyro_x;
    imu_data->angular_velocity.y = gyro_y;
    imu_data->angular_velocity.z = gyro_z;

    // imu_pub->publish(imu_data);
    imu_queue.Push(imu_data);
}

void hotplug_callback(bool bArrive)
{
}

void open_cam()
{
    SetHotplugCallback(&hotplug_callback);

    di = DriverFactory();

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


    di->SetCameraCallback(&cam_callback);
    di->SetIMUCallback(&imu_callback);
    bool sc = di->Open(1000, 50, indem::IMAGE_RESOLUTION::RESOLUTION_640); // imu hz,img hz
    std::cout << "open device " << (sc ? "successful" : "failed") << std::endl;
}

class CameraImuPublisher : public rclcpp::Node
{

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam0_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam1_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread worker_thread_;
    std::atomic<bool> is_running_;

public:
    CameraImuPublisher() : Node("data_publisher")
    {
        cam0_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", 10);
        cam1_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/cam1/image_raw", 10);
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu0", 1000);

        is_running_ = true;
        worker_thread_ = std::thread(&CameraImuPublisher::publish_data, this);

        RCLCPP_INFO(this->get_logger(), "Data publisher node initialized.");
    }

    ~CameraImuPublisher()
    {
        is_running_ = false;

        if (worker_thread_.joinable())
        {
            worker_thread_.join();
            RCLCPP_INFO(this->get_logger(), "Data publisher node thread exit.");
        }
    }

    void publish_data()
    {
        while (is_running_)
        {
            bool published = false;
            if (cam0_queue.Size() > 0)
            {
                cam0_image_pub->publish(*(cam0_queue.Pop()->toImageMsg()));
                published = true;
            }
            if (cam1_queue.Size() > 0)
            {
                cam1_image_pub->publish(*(cam1_queue.Pop()->toImageMsg()));
                published = true;
            }

            if (imu_queue.Size() > 0)
            {
                // std::cout << "imu queue size:" << imu_queue.Size() << std::endl;
                auto imu_data = *(imu_queue.Pop().get());

                imu_pub->publish(imu_data);
                published = true;
            }
            if(!published)
            {
                usleep(100);
            }
        }
    }
};

int main(int argc, char *argv[])
{
    std::thread oc = std::thread(&open_cam);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraImuPublisher>());
    rclcpp::shutdown();
    di->Close();
    oc.join();

    return 0;
}

