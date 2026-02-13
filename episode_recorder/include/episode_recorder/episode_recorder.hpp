
#ifndef EPISODE_RECORDER__EPISODE_RECORDER_HPP_
#define EPISODE_RECORDER__EPISODE_RECORDER_HPP_

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace episode_recorder {

class EpisodeRecorder : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EpisodeRecorder(const rclcpp::NodeOptions &options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // LifeCycle Callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

private:
  // ROS2 Parameters
  std::string root_dir_;
  std::string storage_id_;
  std::vector<std::string> topics_;
  double max_episode_duration_{0.0};
  std::string experiment_name_;

  // Effective output path: root_dir_ / experiment_name_ (if set)
  std::filesystem::path output_dir_;

  // Topics and Subscriptions
  std::unordered_map<std::string, std::string> topic_type_map_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  void resolve_topic_types();
  void create_subscriptions();
  rclcpp::QoS qos_for_topic(const std::string &topic) const;

  // Recording State
  std::mutex recording_mutex_;
  std::atomic<bool> is_recording_{false};
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Time episode_start_time_{0, 0, RCL_SYSTEM_TIME};
  std::filesystem::path current_episode_dir_;
  rclcpp::Clock bag_clock_{RCL_SYSTEM_TIME};

  // Episode Management
  uint32_t next_episode_index_{0};

  uint32_t scan_existing_episodes(const std::filesystem::path &dir) const;
  std::filesystem::path make_episode_dir(uint32_t index) const;

  // Max Duration timer
  rclcpp::TimerBase::SharedPtr duration_timer_;
  void on_max_duration_reached();

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr discard_service_;

  void handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_discard(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Lifecycle guard
  bool cleaned_up_{false};

  // Internal helpers
  bool start_episode();
  bool stop_episode();
  bool discard_episode();
  void on_message_received(const std::string &topic, const std::string &type,
                           std::shared_ptr<rclcpp::SerializedMessage> message);
};

} // namespace episode_recorder

#endif // EPISODE_RECORDER__EPISODE_RECORDER_HPP_