#include <control_msgs/action/parallel_gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class ArmGripperTeleop : public rclcpp::Node
{
public:
  using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;

  ArmGripperTeleop() : Node("arm_gripper_teleop") {
    RCLCPP_INFO(get_logger(), "Initializing ArmGripperTeleop...");

    // Parameters
    // Arm output mode:
    //  - "joint_trajectory" => JointTrajectoryController topic
    //  - "forward_position" => ForwardController commands topic
    arm_mode_ = this->declare_parameter<std::string>("arm_mode", "joint_trajectory");

    leader_topic_ = declare_parameter<std::string>("leader_topic", "/leader/joint_states");
    follower_jtc_topic_ = declare_parameter<std::string>(
        "jtc_topic", "/follower/arm_trajectory_controller/joint_trajectory");
    follower_fwd_topic_ =
        declare_parameter<std::string>("fwd_topic", "/follower/arm_forward_controller/commands");
    gripper_action_name_ = declare_parameter<std::string>(
        "gripper_action", "/follower/gripper_controller/gripper_cmd");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.25);
    point_dt_s_ = declare_parameter<double>("point_dt_s", 0.08);
    gripper_deadband_ = declare_parameter<double>("gripper_deadband", 0.005);
    gripper_min_interval_s_ = declare_parameter<double>("gripper_min_interval_s", 0.05);

    arm_joints_ = declare_parameter<std::vector<std::string>>(
        "arm_joints", std::vector<std::string>{"shoulder_pan", "shoulder_lift", "elbow_flex",
                                               "wrist_flex", "wrist_roll"});
    gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper");

    RCLCPP_INFO(get_logger(), "Leader: %s", leader_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Follower JTC: %s", follower_jtc_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Gripper action: %s", gripper_action_name_.c_str());
    RCLCPP_INFO(get_logger(), "Rate: %.1f Hz, Arm joints: %zu", publish_rate_hz_,
                arm_joints_.size());

    // ROS interfaces
    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        leader_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ArmGripperTeleop::joint_state_callback, this, std::placeholders::_1));

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        follower_jtc_topic_, rclcpp::QoS(10).reliable());
    forward_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(follower_fwd_topic_,
                                                                      rclcpp::QoS(10).reliable());
    gripper_action_client_ =
        rclcpp_action::create_client<ParallelGripperCommand>(this, gripper_action_name_);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                               std::bind(&ArmGripperTeleop::control_loop, this));

    raw_arm_.resize(arm_joints_.size(), 0.0);

    RCLCPP_INFO(get_logger(), "ArmGripperTeleop initialized.");
  }

private:
  // Parameters
  std::string arm_mode_;
  std::string leader_topic_;
  std::string follower_jtc_topic_;
  std::string follower_fwd_topic_;
  std::string gripper_action_name_;
  double publish_rate_hz_{50.0};
  double stale_timeout_s_{0.25};
  double point_dt_s_{0.02};
  double gripper_deadband_{0.005};
  double gripper_min_interval_s_{0.05};
  std::vector<std::string> arm_joints_;
  std::string gripper_joint_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool initialized_{false};
  std::vector<int> arm_idx_;
  std::optional<int> gripper_idx_;
  std::vector<double> raw_arm_;
  double raw_gripper_{0.0};
  rclcpp::Time last_leader_stamp_{0, 0, RCL_ROS_TIME};
  double last_gripper_goal_{0.0};
  rclcpp::Time last_gripper_goal_time_{0, 0, RCL_ROS_TIME};

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    if (!initialized_ && !initialize_indices(*msg)) return;

    last_leader_stamp_ = this->now();

    // Cache targets
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      raw_arm_[i] = msg->position[arm_idx_[i]];
    }
    if (gripper_idx_) raw_gripper_ = msg->position[*gripper_idx_];
  }

  bool initialize_indices(const sensor_msgs::msg::JointState &msg) {
    // Build name -> index map
    std::unordered_map<std::string, int> idx_map;
    idx_map.reserve(msg.name.size());
    for (size_t i = 0; i < msg.name.size(); i++)
      idx_map[msg.name[i]] = static_cast<int>(i);

    // Arm indexes
    arm_idx_.assign(arm_joints_.size(), -1);
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      auto it = idx_map.find(arm_joints_[i]);
      if (it == idx_map.end()) {
        RCLCPP_ERROR(this->get_logger(), "Leader arm joint '%s' not found", arm_joints_[i].c_str());
        return false;
      }
      arm_idx_[i] = it->second;
    }

    // Gripper index
    if (auto it = idx_map.find(gripper_joint_); it != idx_map.end()) {
      gripper_idx_ = it->second;
    } else {
      RCLCPP_WARN(get_logger(), "Leader gripper '%s' not found", gripper_joint_.c_str());
    }

    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initialized: %zu arm joints", arm_joints_.size());
    return true;
  }

  void control_loop() {

    if (!initialized_) return;

    const auto now = this->now();

    // Leader data stale: do nothing (holds last command on follower)
    if ((now - last_leader_stamp_).seconds() > stale_timeout_s_) return;

    // publish arm and gripper
    publish_arm(now);
    publish_gripper(now);
  }

  void publish_arm(const rclcpp::Time &time) {
    if (arm_mode_ == "joint_trajectory") {
      trajectory_msgs::msg::JointTrajectory jt;
      jt.header.stamp = time;
      jt.joint_names = arm_joints_;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = raw_arm_;

      const int sec = static_cast<int>(point_dt_s_);
      const int nsec = static_cast<int>((point_dt_s_ - sec) * 1e9);
      pt.time_from_start.sec = sec;
      pt.time_from_start.nanosec = nsec;
      jt.points.push_back(pt);
      trajectory_pub_->publish(jt);
    } else {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = raw_arm_;
      forward_pub_->publish(cmd);
    }
  }

  void publish_gripper(const rclcpp::Time &time) {
    if (!gripper_idx_) return;
    if (std::abs(raw_gripper_ - last_gripper_goal_) <= gripper_deadband_) return;
    if ((time - last_gripper_goal_time_).seconds() < gripper_min_interval_s_) return;
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Gripper action server not ready (%s)",
                           gripper_action_name_.c_str());
      return;
    }

    ParallelGripperCommand::Goal goal;
    goal.command.name = {gripper_joint_};
    goal.command.position = {raw_gripper_};
    gripper_action_client_->async_send_goal(goal);
    last_gripper_goal_ = raw_gripper_;
    last_gripper_goal_time_ = time;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmGripperTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}