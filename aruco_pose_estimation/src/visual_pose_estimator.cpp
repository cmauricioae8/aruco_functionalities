/**
 * @file visual_pose_estimator.cpp
 * @brief ROS 2 node for robot pose estimation using ArUco markers.
 * 
 * This node detects ArUco markers from a camera feed (via topic or camera port)
 * and estimates the robot's pose in the 'world' frame based on the known 
 * positions of the markers. It publishes the estimated pose as a 
 * geometry_msgs/msg/PoseStamped message, on topic 'visual_robot_pose'.
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <map>
#include <string>

using namespace cv;

class VisualPoseEstimator : public rclcpp::Node {
public:
    VisualPoseEstimator() : Node("visual_pose_estimator") {
        // Declare Parameters
        this->declare_parameter("image_topic", "/camera/image_raw");
        this->declare_parameter("camera_info_topic", "/camera/camera_info");
        this->declare_parameter("camera_port", -1); // -1 means disabled, use topic
        this->declare_parameter("x_offset_c_b", 0.0);
        this->declare_parameter("marker_size", 0.2);
        this->declare_parameter("ceiling_height", 3.0);
        this->declare_parameter("separation_x", 2.0);
        this->declare_parameter("separation_y", 2.0);
        this->declare_parameter("origin_offset_x", -2.0);
        this->declare_parameter("origin_offset_y", -2.0);
        this->declare_parameter("rows", 5);
        this->declare_parameter("cols", 5);
        this->declare_parameter("start_id", 0);
        this->declare_parameter("frequency", 10.0);
        this->declare_parameter("camera_frame", "camera_link");
        this->declare_parameter("world_frame", "odom");
        this->declare_parameter("debug", false);
        
        // Intrinsics (can be passed as flattened arrays)
        this->declare_parameter("camera_matrix", std::vector<double>{640.0, 0, 320.0, 0, 640.0, 240.0, 0, 0, 1.0});
        this->declare_parameter("dist_coeffs", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

        // Get Parameters
        image_topic_ = this->get_parameter("image_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        camera_port_ = this->get_parameter("camera_port").as_int();
        x_offset_c_b_ = this->get_parameter("x_offset_c_b").as_double();
        marker_size_ = this->get_parameter("marker_size").as_double();
        ceiling_height_ = this->get_parameter("ceiling_height").as_double();
        sep_x_ = this->get_parameter("separation_x").as_double();
        sep_y_ = this->get_parameter("separation_y").as_double();
        off_x_ = this->get_parameter("origin_offset_x").as_double();
        off_y_ = this->get_parameter("origin_offset_y").as_double();
        rows_ = this->get_parameter("rows").as_int();
        cols_ = this->get_parameter("cols").as_int();
        start_id_ = this->get_parameter("start_id").as_int();
        frequency_ = this->get_parameter("frequency").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();
        debug_ = this->get_parameter("debug").as_bool();

        std::vector<double> cam_mat = this->get_parameter("camera_matrix").as_double_array();
        camera_matrix_ = cv::Mat(3, 3, CV_64F, cam_mat.data()).clone();
        
        std::vector<double> dist_coeff = this->get_parameter("dist_coeffs").as_double_array();
        dist_coeffs_ = cv::Mat(dist_coeff.size(), 1, CV_64F, dist_coeff.data()).clone();

        got_camera_info_ = true;

        // Initialize ArUco dictionary (7x7_1000 as per simulation)
        RCLCPP_INFO(this->get_logger(), "arUco dictionary: DICT_7X7_1000");
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
        parameters_ = cv::aruco::DetectorParameters::create();

        // Compute marker world positions
        initMarkerMap();

        // Publisher for robot pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("visual_robot_pose", 10);

        // Publisher for debug image
        debug_pub_ = image_transport::create_publisher(this, "~/debug_markers");

        // Parameter Callback
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&VisualPoseEstimator::onParameterChange, this, std::placeholders::_1));

        // Setup Input
        if (camera_port_ >= 0) {
            RCLCPP_INFO(this->get_logger(), "Opening camera port %d at %.2f Hz", camera_port_, frequency_);
            cap_.open(camera_port_);
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera port %d", camera_port_);
                rclcpp::shutdown();
            } else {
                int period_ms = static_cast<int>(1000.0 / frequency_);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&VisualPoseEstimator::onTimer, this));
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Since camera_port < 0, subscribing to image topic: %s", image_topic_.c_str());
            sub_ = image_transport::create_subscription(this, image_topic_, 
                std::bind(&VisualPoseEstimator::onImage, this, std::placeholders::_1), "raw");
            
            // Subscribe to CameraInfo
            got_camera_info_ = false;
            cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, 10, std::bind(&VisualPoseEstimator::onCameraInfo, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to camera info topic: %s", camera_info_topic_.c_str());
        }
    }

private:
    void initMarkerMap() {
        for (int r = 0; r < rows_; ++r) {
            for (int c = 0; c < cols_; ++c) {
                int id = start_id_ + r * cols_ + c;
                cv::Point3f pos;
                pos.x = c * sep_x_ + off_x_;
                pos.y = r * sep_y_ + off_y_;
                pos.z = ceiling_height_;
                marker_map_[id] = pos;
            }
        }
    }

    // Callback for timer (when camera_port >= 0)
    void onTimer() {
        cv::Mat frame;
        cap_ >> frame;
        if (!frame.empty()) {
            processImage(frame, this->now());
        }
    }

    // Callback for image topic subscription (when camera_port < 0)
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            processImage(cv_ptr->image, msg->header.stamp);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    bool isRotationMatrix(Mat &R)
    {
        Mat Rt;
        transpose(R, Rt);
        Mat shouldBeIdentity = Rt * R;
        Mat I = Mat::eye(3,3, shouldBeIdentity.type());
        return  norm(I, shouldBeIdentity) < 1e-6;
    }

    // Calculates rotation matrix to euler angles. The result is the same as MATLAB except the
    // order of the euler angles ( x and z are swapped ).
    Vec3f rotationMatrixToEulerAngles(Mat &R)
    {
        assert(isRotationMatrix(R));

        float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

        bool singular = sy < 1e-6; // If

        float x, y, z;
        if (!singular)
        {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        }
        else
        {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }
        return Vec3f(x, y, z);
    }

    void processImage(cv::Mat& image, rclcpp::Time stamp) {
        if (!got_camera_info_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for camera info...");
            return;
        }
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        // Flip image upside down (this operation is mandatory in order to get the image in the correct orientation)
        cv::flip(image, image, -1); 
        cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_);

        if(debug_){
            // Debug Marker Image
            cv::Mat debug_frame = image.clone();
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(debug_frame, corners, ids);

                // Draw horizontal and vertical camera axes
                cv::Point2f center(image.cols / 2, image.rows / 2);
                cv::Point2f horizontal(image.cols, image.rows / 2);
                cv::Point2f vertical(image.cols / 2, image.rows);
                cv::line(debug_frame, center, horizontal, cv::Scalar(0, 0, 0255), 2);
                cv::line(debug_frame, center, vertical, cv::Scalar(0, 255, 0), 2);
            }
            
            // Publish debug image
            auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_frame).toImageMsg();
            debug_msg->header.stamp = stamp;
            debug_msg->header.frame_id = camera_frame_; 
            debug_pub_.publish(debug_msg);
        }

        if (ids.empty()) return;

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        // Simple averaging of base position derived from each detected marker
        cv::Vec3d avg_base_pos(0,0,0);
        tf2::Quaternion avg_base_q(0, 0, 0, 1); 
        int valid_count = 0;

        for (size_t i = 0; i < ids.size(); ++i) {
            if (marker_map_.find(ids[i]) == marker_map_.end()) continue;

            // Transformation from camera to marker: T_c_m
            cv::Mat R_c_m;
            cv::Rodrigues(rvecs[i], R_c_m);

            cv::Mat T_c_m = cv::Mat::eye(4, 4, CV_64F);
            R_c_m.copyTo(T_c_m(cv::Rect(0, 0, 3, 3)));
            T_c_m.at<double>(0, 3) = tvecs[i][0];
            T_c_m.at<double>(1, 3) = tvecs[i][1];
            T_c_m.at<double>(2, 3) = tvecs[i][2];
            if(debug_){
                RCLCPP_INFO(this->get_logger(), "\n\n-------------");
                RCLCPP_INFO(this->get_logger(), "T_c_m: (x,y,z): (%.3f,%.3f,%.3f)", T_c_m.at<double>(0, 3), T_c_m.at<double>(1, 3), T_c_m.at<double>(2, 3));
            }
            
            // Compute yaw marker with respect camera frame getting the euler angles
            cv::Mat Rotationmatrix;
            cv::Rodrigues(rvecs[i], Rotationmatrix);//Rodrigues converts rvec into the rotation matrix R (and vice versa)

            cv::Vec3f attitude;
            attitude = rotationMatrixToEulerAngles(Rotationmatrix);
            double th_c_m_aux = attitude[2] + M_PI_2;
            double th_c_m = atan2(sin(th_c_m_aux),cos(th_c_m_aux));
            if(debug_) RCLCPP_INFO(this->get_logger(), "Yaw_c_m: %.3f", th_c_m);
            
            // So far, T_c_m with Z_th correct

            // Consider offset between camera and base_footprint (assuming 90° rotation from b to c)
            double x_b_m = -tvecs[i][1] + x_offset_c_b_;
            double y_b_m = tvecs[i][0]; 

            double th_b_m_aux = th_c_m - M_PI_2; 
            double th_b_m = atan2(sin(th_b_m_aux),cos(th_b_m_aux)); // to get -M_PI < th < M_PI
            if(debug_) RCLCPP_INFO(this->get_logger(), "T_b_m: (x,y,th): (%.3f,%.3f,%.3f)", x_b_m, y_b_m, th_b_m);

            // Marker position in world
            cv::Point3f marker_w = marker_map_[ids[i]];

            double th_w_b = -th_b_m;
            double th_w_b_pi = th_w_b+M_PI;
            
            double X_w_b = x_b_m*cos(th_w_b_pi) - y_b_m*sin(th_w_b_pi) + marker_w.x;
            double Y_w_b = x_b_m*sin(th_w_b_pi) + y_b_m*cos(th_w_b_pi) + marker_w.y;

            if(debug_) RCLCPP_INFO(this->get_logger(), "T_w_b: (x,y,th): (%.3f,%.3f,%.3f)", X_w_b, Y_w_b, th_w_b);
            
            // Transformation from world (odom) to marker (facing down)
            avg_base_q.setRPY(0,0,th_w_b);
            
            avg_base_pos[0] += X_w_b;
            avg_base_pos[1] += Y_w_b;

            valid_count++;
        }

        if (valid_count > 0) {
            avg_base_pos /= (double)valid_count;
            avg_base_q.normalize();

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = world_frame_;
            pose_msg.pose.position.x = avg_base_pos[0];
            pose_msg.pose.position.y = avg_base_pos[1];
            pose_msg.pose.position.z = 0.0;
            pose_msg.pose.orientation = tf2::toMsg(avg_base_q);

            pose_pub_->publish(pose_msg);
        }
    }

    // Parameters
    std::string image_topic_, camera_info_topic_, camera_frame_, world_frame_;
    int camera_port_;
    double marker_size_;
    double ceiling_height_;
    double sep_x_, sep_y_, off_x_, off_y_, x_offset_c_b_;
    int rows_, cols_, start_id_;
    double frequency_;
    bool debug_;
    bool got_camera_info_;
    cv::Mat camera_matrix_, dist_coeffs_;

    // OpenCV
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::VideoCapture cap_;

    // ROS
    image_transport::Subscriber sub_;
    image_transport::Publisher debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::map<int, cv::Point3f> marker_map_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : parameters) {
            if (param.get_name() == "debug") {
                debug_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "Debug mode %s", debug_ ? "ENABLED" : "DISABLED");
            }
        }
        return result;
    }

    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (got_camera_info_) return; // Only need it once or if we want dynamic updates, remove this line

        camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, (void*)msg->d.data()).clone();
        
        got_camera_info_ = true;
        RCLCPP_INFO(this->get_logger(), "Received camera info. Matrix size: %dx%d, Distortion coeffs count: %zu", 
                    camera_matrix_.rows, camera_matrix_.cols, msg->d.size());
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}
