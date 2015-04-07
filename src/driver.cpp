#include <libfreenect.h>

#include <iostream>

#include <opencv2/core/core.hpp>

#include <rgbd/Server.h>

#include <ros/init.h>
#include <ros/node_handle.h>

cv::Mat depth_image;
cv::Mat rgb_image;

bool new_image_ = false;

// ----------------------------------------------------------------------------------------------------

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    uint16_t *depth = (uint16_t*)v_depth;

    for(int i = 0; i < depth_image.rows * depth_image.cols; ++i)
    {
        float d;
        if (depth[i] > 0)
            d = depth[i];// / 1000;
        else
            d = 0;
        //            std::cout << d << std::endl;
        depth_image.at<float>(i) = d;
    }


    depth_image = depth_image / 1000;
}

// ----------------------------------------------------------------------------------------------------

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_driver");

    ros::NodeHandle nh_private("~");
    std::string frame_id = "rgbd";
    nh_private.getParam("frame_id", frame_id);

    double fx = 538.6725257330964;
    double fy = 502.5794530135827;
    bool high_resolution = false;
    bool verbose = false;
    nh_private.getParam("fx", fx);
    nh_private.getParam("fy", fy);
    nh_private.getParam("high_resolution", high_resolution);
    nh_private.getParam("verbose", verbose);

    freenect_context *f_ctx;
    freenect_device *f_dev;

    // Initialize freenect
    if (freenect_init(&f_ctx, NULL) < 0)
    {
        std::cout << "freenect_init() failed" << std::endl;
        return 1;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));

    int nr_devices = freenect_num_devices (f_ctx);
    std::cout << "[KINECT DRIVER] Number of devices found: " << nr_devices << std::endl;

    if (nr_devices < 1) {
        freenect_shutdown(f_ctx);
        return 1;
    }

    std::cout << "[KINECT DRIVER] Opening device..." << std::endl;

    int user_device_number = 0;
    if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0)
    {
        std::cout << "[KINECT DIVER] Could not open device" << std::endl;
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

    std::cout << "[KINECT DRIVER] Listening to video stream" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - -

    rgbd::Server server;
    server.initialize("rgbd", rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_PNG);

    geo::DepthCamera cam_model;
    cam_model.setFocalLengths(fx, fy);
    cam_model.setOpticalTranslation(0, 0);
    cam_model.setOpticalCenter(319.5, 239.5);

    std::cout << "[KINECT DRIVER] Up and running" << std::endl;

    while (ros::ok())
    {
        new_image_ = false;

        int res = freenect_process_events(f_ctx);
        if (res < 0 && res != -10)
        {
            std::cout << "\nError " << res << " received from libusb - aborting." << std::endl;
            return 1;
        }

        if (new_image_)
        {
            rgbd::Image image(rgb_image, depth_image, cam_model, frame_id, ros::Time::now().toSec());
            server.send(image, true);
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -

    std::cout << "[KINECT DRIVER] Driver stopped" << std::endl;

    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
}

