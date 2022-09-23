#include <libfreenect.h>

#include <iostream>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/core.hpp>

#include <rgbd/server.h>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/console.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <kinect_driver/SetSettings.h>


cv::Mat depth_image;
cv::Mat rgb_image;

bool new_image_ = false;

double time_offset = 0;

// ----------------------------------------------------------------------------------------------------

void depth_cb(freenect_device* /*dev*/, void* v_depth, uint32_t /*timestamp*/)
{
    uint16_t *depth = (uint16_t*)v_depth;

    for(int i = 0; i < depth_image.rows * depth_image.cols; ++i)
    {
        float d;
        if (depth[i] > 0)
            d = depth[i];// / 1000;
        else
            d = 0;
        //            ROS_ERROR_STREAM(d);
        depth_image.at<float>(i) = d;
    }

    depth_image = depth_image / 1000;
}

// ----------------------------------------------------------------------------------------------------

void rgb_cb(freenect_device* /*dev*/, void* rgb, uint32_t /*timestamp*/)
{
    uint8_t *rgb_mid = (uint8_t*)rgb;

    int i = 0;
    for(int y = 0; y < rgb_image.rows; ++y)
    {
        for(int x = 0; x < rgb_image.cols; ++x)
        {
            rgb_image.at<cv::Vec3b>(y, x) = cv::Vec3b(rgb_mid[i+2], rgb_mid[i+1], rgb_mid[i]);
            i += 3;
        }
    }

    new_image_ = true;
}

// ----------------------------------------------------------------------------------------------------

bool srvSetSettings(kinect_driver::SetSettings::Request& req, kinect_driver::SetSettings::Response& /*res*/)
{
    time_offset = req.time_offset;
    return true;
}


// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_driver");

    ros::NodeHandle nh_private("~");
    std::string frame_id = "rgbd";
    nh_private.getParam("frame_id", frame_id);

    double fx = 538.6725257330964;
    double fy = 502.5794530135827;
    double cx = 319.5;
    double cy = 239.5;
    bool high_resolution = false;
    bool verbose = false;
    nh_private.getParam("fx", fx);
    nh_private.getParam("fy", fy);
    nh_private.getParam("cx", cx);
    nh_private.getParam("cy", cy);
    nh_private.getParam("high_resolution", high_resolution);
    nh_private.getParam("time_offset", time_offset);
    nh_private.getParam("verbose", verbose);

    ros::ServiceServer srv_set_settings = nh_private.advertiseService("set_settings", srvSetSettings);

    freenect_context *f_ctx;
    freenect_device *f_dev;

    // Initialize freenect
    if (freenect_init(&f_ctx, NULL) < 0)
    {
        ROS_ERROR_STREAM("freenect_init() failed");
        return 1;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));

    int nr_devices = freenect_num_devices (f_ctx);
    ROS_INFO_STREAM("[KINECT DRIVER] Number of devices found: " << nr_devices);

    if (nr_devices < 1) {
        freenect_shutdown(f_ctx);
        return 1;
    }

    ROS_INFO_STREAM("[KINECT DRIVER] Opening device...");

    int user_device_number = 0;
    if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0)
    {
        ROS_ERROR_STREAM("[KINECT DIVER] Could not open device");
        freenect_shutdown(f_ctx);
        return 1;
    }

    freenect_frame_mode video_mode;

    if (high_resolution)
        video_mode = freenect_find_video_mode(FREENECT_RESOLUTION_HIGH, FREENECT_VIDEO_RGB);
    else
        video_mode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);

    freenect_frame_mode depth_mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED);

    rgb_image = cv::Mat(video_mode.height, video_mode.width, CV_8UC3, cv::Scalar(0, 0, 0));
    depth_image = cv::Mat(depth_mode.height, depth_mode.width, CV_32FC1, 0.0);

    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);
    freenect_set_video_mode(f_dev, video_mode);
    freenect_set_depth_mode(f_dev, depth_mode);

    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);

    ROS_INFO_STREAM("[KINECT DRIVER] Listening to video stream");

    // - - - - - - - - - - - - - - - - - - - - - - - -

    rgbd::Server server;
    server.initialize("rgbd", rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_PNG);

    sensor_msgs::CameraInfo cam_info;
    cam_info.K = {fx, 0.0, cx,
                  0.0, fy, 240.5,
                  0.0, 0.0, 1.0};
    cam_info.P = {fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0};
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = video_mode.width;
    cam_info.height = video_mode.height;
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info);

    ROS_INFO_STREAM("[KINECT DRIVER] Up and running");

    while (ros::ok())
    {
        // Check for service requests
        ros::spinOnce();

        new_image_ = false;

        int res;
        try
        {
            res = freenect_process_events(f_ctx);
        } 
        catch (boost::thread_resource_error const &)
        {
            ROS_ERROR_STREAM("[KINECT DRIVER] boost::thread_resource_error catch, continue ...");
            return 1;
        }

        if (res < 0 && res != -10)
        {
            ROS_ERROR_STREAM("[KINECT DRIVER] Error " << res << " received from libusb - aborting.");
            return 1;
        }

        if (new_image_)
        {
            double time = ros::Time::now().toSec() + time_offset;
            rgbd::Image image(rgb_image, depth_image, cam_model, frame_id, time);
            server.send(image, true);
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -

    ROS_ERROR_STREAM("[KINECT DRIVER] Driver stopped");

    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
}
