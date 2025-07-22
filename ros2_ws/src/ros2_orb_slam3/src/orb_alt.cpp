#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // NEW: Include PoseStamped
#include <librealsense2/h/rs_sensor.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdlib.h>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>

#include <System.h>

using namespace std::chrono_literals;

static rs2_option get_sensor_option(const rs2::sensor &sensor)
{
    // Sensors usually have several options to control their properties
    // such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
        rs2_option option_type = static_cast<rs2_option>(i);
        // SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type)) {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char *description = sensor.get_option_description(option_type);
            std::cout << "        Description    : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "        Current Value : " << current_value << std::endl;

            // To change the value of an option, please follow the
            // change_sensor_option() function
        } else {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

class OrbAlt : public rclcpp::Node {
public:
    OrbAlt() : Node("orb_alt")
    {
        timer_ = create_wall_timer(5ms, std::bind(&OrbAlt::timer_callback, this));

        // declare parameters
        declare_parameter("sensor_type", "imu-monocular");
        declare_parameter("use_pangolin", true);

        // get parameters
        sensor_type_param = get_parameter("sensor_type").as_string();
        use_pangolin = get_parameter("use_pangolin").as_bool();

        // set the sensor type based on parameter
        vocabulary_file_path_ =
            std::string(PROJECT_PATH) + "/ORB_SLAM3/Vocabulary/ORBvoc.txt";
        if (sensor_type_param == "monocular") {
            sensor_type = ORB_SLAM3::System::MONOCULAR;
            settings_file_path_ =
                std::string(PROJECT_PATH) + "/config/Monocular/RealSense_D435i.yaml";
        } else if (sensor_type_param == "imu-monocular") {
            sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
            settings_file_path_ = std::string(PROJECT_PATH) +
                                  "/ORB_SLAM3/Examples/Monocular-Inertial/RealSense_D435i.yaml"; // Corrected path
        } else {
            RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
            rclcpp::shutdown();
        }

        RCLCPP_INFO_STREAM(get_logger(),
                           "vocabulary_file_path: " << vocabulary_file_path_);

        // setup orb slam object
        SLAM = std::make_shared<ORB_SLAM3::System>(
            vocabulary_file_path_, settings_file_path_, sensor_type, use_pangolin, 0);

        // create publishers
        live_point_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("live_point_cloud", 10);
        // orb_image_publisher_ =
        //    create_publisher<sensor_msgs::msg::Image>("/orb_camera/image", 10);

        // NEW: Publisher for camera pose
        camera_pose_publisher_ = 
            create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);

        // tf broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // create timer
        timestamp_ = generate_timestamp_string();

        output_path_ = std::string(PROJECT_PATH) + "/output/" + timestamp_;
        if (!std::filesystem::create_directory(output_path_)) {
            std::cout << "Failed to create output directory" << std::endl;
            return;
        }
        if (!std::filesystem::create_directory(output_path_ + "/cloud")) {
            std::cout << "Failed to create cloud directory" << std::endl;
            return;
        }
        if (!std::filesystem::create_directory(output_path_ + "/images")) {
            std::cout << "Failed to create images directory" << std::endl;
            return;
        }
        if (!std::filesystem::create_directory(output_path_ + "/poses")) {
            std::cout << "Failed to create poses directory" << std::endl;
            return;
        }

        rclcpp::Context::SharedPtr context =
            get_node_base_interface()->get_context();

        auto rcl_preshutdown_cb_handle_ =
            std::make_unique<rclcpp::PreShutdownCallbackHandle>(
                context->add_pre_shutdown_callback(
                    std::bind(&OrbAlt::preshutdown, this)));

        setup_realsense();
    }

private:
    void preshutdown()
    {
        RCLCPP_INFO(get_logger(), "Shutting down ROS 2");
        pcl::PointCloud<pcl::PointXYZ> cloud = SLAM->GetMapPCL();
        RCLCPP_INFO_STREAM(get_logger(), "cloud size: " << cloud.size());
        pcl::io::savePCDFileBinary(output_path_ + "/cloud/" + timestamp_ + ".pcd",
                                   cloud);
        std::ofstream fout(output_path_ + "/poses/" + timestamp_ + ".yaml");
        fout << poses_;
        fout.close();
    }

    rs2_vector interpolate_measure(const double target_time,
                                   const rs2_vector current_data,
                                   const double current_time,
                                   const rs2_vector prev_data,
                                   const double prev_time)
    {

        // If there are not previous information, the current data is propagated
        if (prev_time == 0) {
            return current_data;
        }

        rs2_vector increment;
        rs2_vector value_interp;

        if (target_time > current_time) {
            value_interp = current_data;
        } else if (target_time > prev_time) {
            increment.x = current_data.x - prev_data.x;
            increment.y = current_data.y - prev_data.y;
            increment.z = current_data.z - prev_data.z;

            double factor = (target_time - prev_time) / (current_time - prev_time);

            value_interp.x = prev_data.x + increment.x * factor;
            value_interp.y = prev_data.y + increment.y * factor;
            value_interp.z = prev_data.z + increment.z * factor;

            // zero interpolation
            value_interp = current_data;
        } else {
            value_interp = prev_data;
        }

        return value_interp;
    }

    void setup_realsense()
    {
        int index = 0;

        devices = ctx.query_devices();
        if (devices.size() == 0) {
            std::cerr << "No device connected, please connect a RealSense device"
                      << std::endl;
            rclcpp::shutdown();
        } else
            selected_device = devices[0];

        sensors = selected_device.query_sensors();
        // We can now iterate the sensors and print their names
        for (rs2::sensor sensor : sensors) {
            if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
                ++index;
                RCLCPP_INFO_STREAM(get_logger(), "sensorfdsa: " << sensor.get_info(
                                       RS2_CAMERA_INFO_NAME));
                RCLCPP_INFO_STREAM(get_logger(), "index: " << index);
                if (index == 1) {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                    sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 5000);
                    sensor.set_option(RS2_OPTION_EMITTER_ENABLED,
                                      0); // switch off emitter
                }
                // std::cout << "  " << index << " : " <<
                // sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
                get_sensor_option(sensor);
                if (index == 2) {
                    // RGB camera (not used here...)
                    sensor.set_option(RS2_OPTION_EXPOSURE, 100.f);
                }

                if (index == 3) {
                    sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
                }
            }
        }

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        // Create a configuration for configuring the pipeline with a non default
        // profile
        // Enabling the depth stream and using it for the mono8 image is faster, and
        // doesn't require a conversion from RGB to mono8 in the future.
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

        // IMU callback
        auto imu_callback = [&](const rs2::frame &frame) {
            std::unique_lock<std::mutex> lock(imu_mutex);

            if (rs2::frameset fs = frame.as<rs2::frameset>()) {
                count_im_buffer++;

                double new_timestamp_image = fs.get_timestamp() * 1e-3;
                if (abs(timestamp_image - new_timestamp_image) < 0.001) {
                    // cout << "Two frames with the same timeStamp!!!\n";
                    count_im_buffer--;
                    return;
                }

                rs2::video_frame color_frame = fs.get_color_frame();
                rs2::video_frame infrared_frame = fs.get_infrared_frame();
                imCV_color =
                    cv::Mat(cv::Size(width_img, height_img), CV_8UC3,
                            (void *)(color_frame.get_data()), cv::Mat::AUTO_STEP);
                imCV = cv::Mat(cv::Size(width_img, height_img), CV_8U,
                               (void *)(infrared_frame.get_data()), cv::Mat::AUTO_STEP);

                timestamp_image = fs.get_timestamp() * 1e-3;
                image_ready = true;

                while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {

                    int index = v_accel_timestamp_sync.size();
                    double target_time = v_gyro_timestamp[index];

                    v_accel_data_sync.push_back(current_accel_data);
                    v_accel_timestamp_sync.push_back(target_time);
                }

                lock.unlock();
                cond_image_rec.notify_all();
            } else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
                if (m_frame.get_profile().stream_name() == "Gyro") {
                    // It runs at 200Hz
                    v_gyro_data.push_back(m_frame.get_motion_data());
                    v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
                    // rs2_vector gyro_sample = m_frame.get_motion_data();
                    // std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y <<
                    // ", " << gyro_sample.z << std::endl;
                } else if (m_frame.get_profile().stream_name() == "Accel") {
                    // It runs at 60Hz
                    prev_accel_timestamp = current_accel_timestamp;
                    prev_accel_data = current_accel_data;

                    current_accel_data = m_frame.get_motion_data();
                    current_accel_timestamp = (m_frame.get_timestamp() + offset) * 1e-3;

                    while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
                        int index = v_accel_timestamp_sync.size();
                        double target_time = v_gyro_timestamp[index];

                        rs2_vector interp_data = interpolate_measure(
                            target_time, current_accel_data, current_accel_timestamp,
                            prev_accel_data, prev_accel_timestamp);

                        v_accel_data_sync.push_back(interp_data);
                        v_accel_timestamp_sync.push_back(target_time);
                    }
                    // std::cout << "Accel:" << current_accel_data.x << ", " <<
                    // current_accel_data.y << ", " << current_accel_data.z << std::endl;
                }
            }
        };

        pipe_profile = pipe.start(cfg, imu_callback);

        cam_stream = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);

        rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);

        rs2_intrinsics intrinsics_cam =
            cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
        width_img = intrinsics_cam.width;
        height_img = intrinsics_cam.height;
        imageScale = SLAM->GetImageScale();

        // Clear IMU vectors
        v_gyro_data.clear();
        v_gyro_timestamp.clear();
        v_accel_data_sync.clear();
        v_accel_timestamp_sync.clear();
    }

    std::string generate_timestamp_string()
    {
        std::time_t now = std::time(nullptr);
        std::tm *ptm = std::localtime(&now);

        std::ostringstream oss;

        oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S");

        return oss.str();
    }

    void timer_callback()
    {

        double timestamp;
        cv::Mat im;

        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if (!image_ready)
                cond_image_rec.wait(lk);

            if (count_im_buffer > 1)
                cout << count_im_buffer - 1 << " dropped frs\n";
            count_im_buffer = 0;

            while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                rs2_vector interp_data = interpolate_measure(
                    target_time, current_accel_data, current_accel_timestamp,
                    prev_accel_data, prev_accel_timestamp);

                v_accel_data_sync.push_back(interp_data);
                // v_accel_data_sync.push_back(current_accel_data); // 0 interpolation
                v_accel_timestamp_sync.push_back(target_time);
            }

            // Copy the IMU data
            vGyro = v_gyro_data;
            vGyro_times = v_gyro_timestamp;
            vAccel = v_accel_data_sync;
            vAccel_times = v_accel_timestamp_sync;
            timestamp = timestamp_image;
            im = imCV.clone();

            // Clear IMU vectors
            v_gyro_data.clear();
            v_gyro_timestamp.clear();
            v_accel_data_sync.clear();
            v_accel_timestamp_sync.clear();

            image_ready = false;
        }

        for (int i = 0; i < vGyro.size(); ++i) {
            ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z,
                                            vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                            vGyro_times[i]);
            vImuMeas.push_back(lastPoint);
        }

        if (imageScale != 1.f) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Pass the image to the SLAM system
        std::shared_ptr<Sophus::SE3f> Tcw;
        if (sensor_type == ORB_SLAM3::System::MONOCULAR) {
            Tcw = std::make_shared<Sophus::SE3f>(SLAM->TrackMonocular(im, timestamp));
        } else if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR) {
            Tcw = std::make_shared<Sophus::SE3f>(
                SLAM->TrackMonocular(im, timestamp, vImuMeas));
        }

        // NEW: Publish camera pose to ROS topic
        if (Tcw) { // Check if tracking was successful and Tcw is valid
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = get_clock()->now();
            pose_msg.header.frame_id = "odom"; // ORB_SLAM3's global frame is typically 'world' or 'odom'

            // Convert Sophus::SE3f (Tcw: camera to world) to Twc (world to camera)
            // and then to geometry_msgs::msg::PoseStamped (Twc)
            Sophus::SE3f Twc_sophus = Tcw->inverse();

            // Position (translation part of Twc)
            pose_msg.pose.position.x = Twc_sophus.translation().x();
            pose_msg.pose.position.y = Twc_sophus.translation().y();
            pose_msg.pose.position.z = Twc_sophus.translation().z();

            // Orientation (rotation part of Twc)
            Eigen::Quaternionf q_eigen = Twc_sophus.unit_quaternion();
            pose_msg.pose.orientation.x = q_eigen.x();
            pose_msg.pose.orientation.y = q_eigen.y();
            pose_msg.pose.orientation.z = q_eigen.z();
            pose_msg.pose.orientation.w = q_eigen.w();

            camera_pose_publisher_->publish(pose_msg);
        } else {
            RCLCPP_WARN(get_logger(), "ORB_SLAM3 did not return a valid pose.");
        }


        // save image
        // cv::Mat pretty = SLAM->getPrettyFrame();
        if (!imCV_color.empty()) {
            cv::imwrite(output_path_ + "/images/" + std::to_string(img_iter_) +
                        ".jpg",
                        imCV_color);
        }

        // save pose to YAML (existing functionality)
        Eigen::Matrix4f transformation_matrix = Tcw->inverse().matrix();
        YAML::Node matrix;
        for (int i = 0; i < 4; i++) {
            std::vector<float> row;
            for (int j = 0; j < 4; j++) {
                row.push_back(transformation_matrix(i, j));
            }
            matrix.push_back(row);
        }
        poses_["Twc_" + std::to_string(img_iter_)] = matrix;
        img_iter_++;

        // Clear the previous IMU measurements to load the new ones
        vImuMeas.clear();
        // }
    }

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseArray pose_array_;
    std::string sensor_type_param;
    bool use_pangolin;

    std::shared_ptr<ORB_SLAM3::System> SLAM;
    std::string vocabulary_file_path_;
    std::string settings_file_path_;

    std::string timestamp_;
    std::string output_path_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        live_point_cloud_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        pose_array_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr orb_image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_publisher_; // NEW: PoseStamped Publisher

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    sensor_msgs::msg::PointCloud2 live_pcl_cloud_msg_;
    pcl::PointCloud<pcl::PointXYZ> live_pcl_cloud_;

    rs2::context ctx;
    rs2::device_list devices;
    rs2::device selected_device;
    std::vector<rs2::sensor> sensors;
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile pipe_profile;

    std::mutex imu_mutex;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    ORB_SLAM3::System::eSensor sensor_type;
    std::condition_variable cond_image_rec;
    vector<double> v_accel_timestamp;
    vector<rs2_vector> v_accel_data;
    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;
    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;
    vector<double> v_accel_timestamp_sync;
    vector<rs2_vector> v_accel_data_sync;
    rs2::stream_profile cam_stream;

    std::vector<rs2_vector> vGyro;
    std::vector<double> vGyro_times;
    std::vector<rs2_vector> vAccel;
    std::vector<double> vAccel_times;

    cv::Mat imCV;
    cv::Mat imCV_color;
    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames
    float imageScale;

    double offset = 0; // ms
    //
    YAML::Node poses_;
    int img_iter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrbAlt>());
    rclcpp::shutdown();
    return 0;
}
