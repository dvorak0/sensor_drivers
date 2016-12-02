#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <queue>

#include <dji_sdk/TimeStamp.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/Acceleration.h>
#include <errno.h>
#include <unistd.h>

#include "camera.h"

using namespace std;

ros::Publisher pub_img, pub_imu;

int img_cnt = 0, trigger_cnt = 0;

BluefoxManager bluefoxManager;

queue<dji_sdk::TimeStampConstPtr> trigger_buf;
int base_tick = -1;
ros::Time base_time;

ros::Time tick2time(int tick)
{
    if (base_tick < 0)
    {
        base_tick = tick;
        base_time = ros::Time::now();
    }
    return base_time + ros::Duration((tick - base_tick) / 400.0);
}

void dji_sdk_callback(const dji_sdk::TimeStampConstPtr &timestamp_msg,
                      const dji_sdk::AccelerationConstPtr &acc_msg,
                      const dji_sdk::AttitudeQuaternionConstPtr &gyr_msg)
{
    if (timestamp_msg->sync_flag)
        trigger_buf.push(timestamp_msg);
    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
    imu_msg->header.stamp = tick2time(timestamp_msg->time);
    imu_msg->linear_acceleration.x = acc_msg->ax * 9.8;
    imu_msg->linear_acceleration.y = acc_msg->ay * 9.8;
    imu_msg->linear_acceleration.z = acc_msg->az * 9.8;
    imu_msg->angular_velocity.x = gyr_msg->wx;
    imu_msg->angular_velocity.y = gyr_msg->wy;
    imu_msg->angular_velocity.z = gyr_msg->wz;
    pub_imu.publish(imu_msg);
}

void process()
{
    vector<vector<char>> data;
    if (!trigger_buf.empty() && bluefoxManager.ready())
    {
        sensor_msgs::ImagePtr img_msg(new sensor_msgs::Image);
        img_msg->height = 1024 * bluefoxManager.getImgCnt();
        img_msg->width = 1280;
        img_msg->step = 1280;
        img_msg->encoding = sensor_msgs::image_encodings::MONO8;

        printf("imu size: %lu\n", trigger_buf.size());
        img_msg->header.stamp = tick2time(trigger_buf.front()->time);
        trigger_buf.pop();
        img_msg->data = bluefoxManager.getImg();
        pub_img.publish(img_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluefox2");
    ros::NodeHandle n("~");

    pub_img = n.advertise<sensor_msgs::Image>("image", 1000);
    pub_imu = n.advertise<sensor_msgs::Imu>("imu", 1000);

    message_filters::Subscriber<dji_sdk::TimeStamp> dji_timestamp(n, "/dji_sdk/time_stamp", 100, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<dji_sdk::Acceleration> dji_acc(n, "/dji_sdk/acceleration", 100, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<dji_sdk::AttitudeQuaternion> dji_gyr(n, "/dji_sdk/attitude_quaternion", 100, ros::TransportHints().tcpNoDelay());

    typedef message_filters::sync_policies::ExactTime<dji_sdk::TimeStamp, dji_sdk::Acceleration, dji_sdk::AttitudeQuaternion>
        MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), dji_timestamp, dji_acc, dji_gyr);
    sync.registerCallback(boost::bind(&dji_sdk_callback, _1, _2, _3));

    ros::Rate r(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        process();
        r.sleep();
    }

    return 0;
}
