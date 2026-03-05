/**
 * @file marker_pose_corrector.cpp
 * @brief ROS 2 node to automatically correct robot pose when AMCL and Visual Pose diverge.
 * 
 * This node subscribes to 'amcl_robot_pose' and 'visual_robot_pose'.
 * If the Euclidean distance or orientation difference between them exceeds 
 * specified tolerances, it publishes the visual pose to 'initialpose' 
 * to re-localize AMCL.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>

class MarkerPoseCorrector : public rclcpp::Node {
public:
    MarkerPoseCorrector() : Node("marker_pose_corrector") {
        // Declare Parameters
        this->declare_parameter("distance_tolerance", 0.5);
        this->declare_parameter("orientation_tolerance", 0.5);

        // Get Parameters
        distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();
        orientation_tolerance_ = this->get_parameter("orientation_tolerance").as_double();

        // Subscribers
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "amcl_robot_pose", 10, std::bind(&MarkerPoseCorrector::amclCallback, this, std::placeholders::_1));
        
        visual_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "visual_robot_pose", 10, std::bind(&MarkerPoseCorrector::visualCallback, this, std::placeholders::_1));

        // Publisher
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        // Parameter Callback
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MarkerPoseCorrector::onParameterChange, this, std::placeholders::_1));

        
        RCLCPP_INFO(this->get_logger(), "Marker Pose Corrector initialized with tolerances: Dist=%.2fm, Yaw=%.2frad", 
                    distance_tolerance_, orientation_tolerance_);
    }

private:
    void amclCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        last_amcl_pose_ = *msg;
        has_amcl_ = true;
    }

    void visualCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!has_amcl_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for AMCL pose on 'amcl_robot_pose'...");
            return;
        }

        // Calculate Euclidean Distance
        double dx = msg->pose.position.x - last_amcl_pose_.position.x;
        double dy = msg->pose.position.y - last_amcl_pose_.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        // Calculate Orientation Difference (Yaw)
        double visual_yaw = tf2::getYaw(msg->pose.orientation);
        double amcl_yaw = tf2::getYaw(last_amcl_pose_.orientation);
        
        double diff = visual_yaw - amcl_yaw;
        double yaw_diff = std::abs(std::atan2(std::sin(diff), std::cos(diff)));

        if (distance > distance_tolerance_ || yaw_diff > orientation_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Correction triggered! Current divergence: Dist=%.3fm, YawDiff=%.3frad", 
                        distance, yaw_diff);
            
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.pose.pose = msg->pose;
            
            // Set a default covariance for initial_pose
            // These values are standard for AMCL initialization
            for (int i = 0; i < 36; ++i) pose_msg.pose.covariance[i] = 0.0;
            pose_msg.pose.covariance[0] = 0.25;  // x variance
            pose_msg.pose.covariance[7] = 0.25;  // y variance
            pose_msg.pose.covariance[35] = 0.06; // yaw variance (pi/6)^2

            initial_pose_pub_->publish(pose_msg);
        }
    }

    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "OK. ROS param dentro del intervalo permitido";

        for (const auto& param : parameters) {
            if (param.get_name() == "distance_tolerance") {
                if (param.as_double() > 0.0){
                    distance_tolerance_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "distance_tolerance modificada a: %.3f m", distance_tolerance_); 
                }else{
                    result.successful = false;
                    result.reason = "distance_tolerance debe ser mayor a cero";
                    return result;
                }
            }
            if (param.get_name() == "orientation_tolerance") {
                if( std::abs(param.as_double()) < 1.0 ){
                    orientation_tolerance_ = std::abs(param.as_double());
                    RCLCPP_INFO(this->get_logger(), "orientation_tolerance changed to %.3f", orientation_tolerance_);
                }else{
                    result.successful = false;
                    result.reason = "orientation_tolerance debe estar entre (-1.0, 1.0) rad";
                    return result;
                }
            }
        }
        return result;
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr amcl_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr visual_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    geometry_msgs::msg::Pose last_amcl_pose_;
    bool has_amcl_ = false;
    double distance_tolerance_;
    double orientation_tolerance_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerPoseCorrector>());
    rclcpp::shutdown();
    return 0;
}
