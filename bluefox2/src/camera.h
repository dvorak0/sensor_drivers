#include <iostream>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <errno.h>

namespace bluefox2
{

class Camera
{
public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    ~Camera();
    bool isOK();
    void feedImages();

private:

    // Node handle
    ros::NodeHandle node, pnode;
    // mvIMPACT Acquire device manager
    mvIMPACT::acquire::DeviceManager devMgr;
    // create an interface to the device found
    mvIMPACT::acquire::FunctionInterface *fi[10];
    // establish access to the statistic properties
    mvIMPACT::acquire::Statistics *statistics[10];
    // Image request
    const mvIMPACT::acquire::Request* pRequest[10];
    // Internal parameters that cannot be changed
    bool ok;
    ros::Time capture_time;
    unsigned int devCnt;

    // User specified parameters
    int cam_cnt;
    std::vector<std::string> serial;
    std::vector<unsigned int> ids;

    int pub_cnt;
    std::vector<ros::Publisher> pub_img;
    std::vector<std::string> masks;
    std::vector<sensor_msgs::Image> img_buf;

    bool use_color;
    bool use_hdr;
    bool has_hdr;
    bool use_binning;
    bool use_auto_exposure;
    double fps;
    double gain;
    int  exposure_time_us;

    bool initSingleMVDevice(unsigned int id);
    bool grab_image_data();
};

}


