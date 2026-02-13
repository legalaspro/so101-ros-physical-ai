#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Terminal raw mode — RAII wrapper around termios
// ─────────────────────────────────────────────────────────────────────────────
namespace {

class TerminalRawMode
{
public:
  TerminalRawMode() = default;
  ~TerminalRawMode() { restore(); }

  // Non-copyable, non-movable
  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode &operator=(const TerminalRawMode &) = delete;

  bool enable() {
    if (enabled_) {
      return true;
    }
    if (!::isatty(STDIN_FILENO)) {
      return false;
    }
    if (::tcgetattr(STDIN_FILENO, &orig_) < 0) {
      return false;
    }

    termios raw = orig_;
    // Disable canonical mode, echo, and extended input processing.
    // Keep ISIG so that Ctrl-C delivers SIGINT normally.
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | IEXTEN);
    raw.c_iflag &= ~static_cast<tcflag_t>(IXON | ICRNL);
    // Non-blocking: return immediately if no data
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0) {
      return false;
    }
    enabled_ = true;
    return true;
  }

  void restore() {
    if (!enabled_) {
      return;
    }
    (void)::tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    enabled_ = false;
  }

  /// Read a single byte from stdin. Returns -1 if nothing available.
  int read_byte() const {
    unsigned char ch = 0;
    ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    return (n == 1) ? static_cast<int>(ch) : -1;
  }

private:
  termios orig_{};
  bool enabled_{false};
};
} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Key codes
// ─────────────────────────────────────────────────────────────────────────────
namespace keys {
constexpr int ESC = 27;
constexpr int BACKSPACE_1 = 127; // Most terminals
constexpr int BACKSPACE_2 = 8;   // Some terminals (Ctrl-H)
// Arrow keys are ESC [ A/B/C/D — handled as escape sequences
constexpr char ARROW_RIGHT = 'C';
constexpr char ARROW_LEFT = 'D';
} // namespace keys

namespace episode_recorder {

// ─────────────────────────────────────────────────────────────────────────────
// Keyboard Controller Node
// ─────────────────────────────────────────────────────────────────────────────

class TeleopEpisodeKeyboard : public rclcpp::Node
{
public:
  explicit TeleopEpisodeKeyboard(TerminalRawMode &terminal,
                                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("teleop_episode_keyboard", options), term_(terminal) {

    recorder_prefix_ = this->declare_parameter<std::string>("recorder_prefix", "/episode_recorder");

    std::string start_srv_name = recorder_prefix_ + "/start_recording";
    std::string stop_srv_name = recorder_prefix_ + "/stop_recording";
    std::string discard_srv_name = recorder_prefix_ + "/discard_episode";

    start_client_ = this->create_client<std_srvs::srv::Trigger>(start_srv_name);
    stop_client_ = this->create_client<std_srvs::srv::Trigger>(stop_srv_name);
    discard_client_ = this->create_client<std_srvs::srv::Trigger>(discard_srv_name);

    RCLCPP_INFO(get_logger(), "Service clients:");
    RCLCPP_INFO(get_logger(), "  start:   %s", start_srv_name.c_str());
    RCLCPP_INFO(get_logger(), "  stop:    %s", stop_srv_name.c_str());
    RCLCPP_INFO(get_logger(), "  discard: %s", discard_srv_name.c_str());

    // Timer for polling keyboard input (~50 Hz)
    key_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                         std::bind(&TeleopEpisodeKeyboard::poll_keyboard, this));

    // Timer for updating status display (~4 Hz)
    display_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(250), std::bind(&TeleopEpisodeKeyboard::update_display, this));

    print_help();
  }

private:
  std::string recorder_prefix_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr discard_client_;
  rclcpp::TimerBase::SharedPtr key_timer_;
  rclcpp::TimerBase::SharedPtr display_timer_;

  bool recording_{false};
  bool service_pending_{false};
  int esc_state_{0}; // 0=none, 1=got ESC, 2=got ESC[
  std::string current_episode_;
  std::chrono::steady_clock::time_point rec_start_{};

  TerminalRawMode &term_; // owned by main(); enabled in main()

  void print_help() {
    std::printf("\n");
    std::printf("╔══════════════════════════════════════════╗\n");
    std::printf("║     Episode Recording Controller         ║\n");
    std::printf("╠══════════════════════════════════════════╣\n");
    std::printf("║  → / r  : Start recording                ║\n");
    std::printf("║  ← / s  : Stop recording & save          ║\n");
    std::printf("║  ⌫ / d  : Discard current episode        ║\n");
    std::printf("║  h      : Help                           ║\n");
    std::printf("║  q      : Quit                           ║\n");
    std::printf("╚══════════════════════════════════════════╝\n");
    std::printf("\n");
    std::fflush(stdout);
  }

  void poll_keyboard() {
    int ch = term_.read_byte();
    if (ch < 0) {
      return; // No input available
    }

    // Simple ESC sequence state machine: ESC -> '[' -> code
    if (esc_state_ == 0) {
      if (ch == keys::ESC) {
        esc_state_ = 1;
        return;
      }
    } else if (esc_state_ == 1) {
      if (ch == '[') {
        esc_state_ = 2;
        return;
      }
      esc_state_ = 0; // not an arrow sequence
      // fallthrough: treat `ch` normally if you want (or ignore)
    } else if (esc_state_ == 2) {
      esc_state_ = 0;
      if (ch == keys::ARROW_RIGHT) {
        handle_start();
        return;
      }
      if (ch == keys::ARROW_LEFT) {
        handle_stop();
        return;
      }
      return; // ignore other ESC[
    }

    // Single - Character keys
    switch (ch) {
    case 'r':
    case 'R':
      handle_start();
      break;
    case 's':
    case 'S':
      handle_stop();
      break;
    case 'd':
    case 'D':
    case keys::BACKSPACE_1:
    case keys::BACKSPACE_2:
      handle_discard();
      break;
    case 'q':
    case 'Q':
      request_quit();
      break;
    case 'h':
    case 'H':
    case '?':
      print_help();
      break;
    default:
      break;
    }
  }

  void update_display() {
    if (!recording_) return;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - rec_start_);
    int mm = static_cast<int>(elapsed.count()) / 60;
    int ss = static_cast<int>(elapsed.count()) % 60;

    std::printf("\r  🔴 REC  %s  %02d:%02d  ",
                current_episode_.empty() ? "-" : current_episode_.c_str(), mm, ss);
    std::fflush(stdout);
  }

  // -------------------------------------
  // Action Handlers
  // -------------------------------------

  void handle_start() {
    if (service_pending_) return msg_("⏳ Service call in progress, please wait...");
    if (recording_) return msg_("⚠  Already recording — stop or discard first");

    msg_("▶  Starting recording...");
    call_(start_client_, "start_recording", [this](bool success, const std::string &message) {
      if (success) {
        recording_ = true;
        rec_start_ = std::chrono::steady_clock::now();
        auto slash = message.rfind('/');
        current_episode_ = (slash != std::string::npos) ? message.substr(slash + 1) : message;
        msg_("▶  Recording %s", current_episode_.c_str());
      } else {
        recording_ = false;
        msg_("✗  Start failed: %s", message.c_str());
      }
    });
  }

  void handle_stop() {
    if (service_pending_) return msg_("⏳ Service call in progress, please wait...");
    if (!recording_) return msg_("⚠  Not recording — nothing to stop");

    msg_("⏹  Stopping recording...");
    call_(stop_client_, "stop_recording", [this](bool success, const std::string &message) {
      if (success) {
        msg_("⏹  Saved %s", current_episode_.c_str());
        current_episode_.clear();
      } else {
        msg_("✗  Stop failed: %s", message.c_str());
      }
      recording_ = false;
    });
  }

  void handle_discard() {
    if (service_pending_) return msg_("⏳ Service call in progress, please wait...");
    if (!recording_) return msg_("⚠  Not recording — nothing to discard");

    msg_("🗑  Discarding episode...");
    call_(discard_client_, "discard_episode", [this](bool success, const std::string &message) {
      if (success) {
        msg_("🗑  Discarded %s", current_episode_.c_str());
        current_episode_.clear();
      } else {
        msg_("✗  Discard failed: %s", message.c_str());
      }
      recording_ = false;
    });
  }

  // ------------------------------------
  // Helpers
  // ------------------------------------
  void request_quit() {
    msg_("\n👋 Shutting down keyboard controller...\n");
    key_timer_->cancel();
    display_timer_->cancel();
    term_.restore(); // immediate
    rclcpp::shutdown();
  }

  using ServiceCallback = std::function<void(bool, const std::string &message)>;

  void call_(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
             const std::string &service_label, ServiceCallback on_result) {
    if (!client->service_is_ready()) {
      msg_("✗  Service '%s' not available — is the recorder running?", service_label.c_str());
      return;
    }

    service_pending_ = true;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto self = std::static_pointer_cast<TeleopEpisodeKeyboard>(shared_from_this());
    // The callback-based overload returns a SharedFuture we don't need to hold.
    (void)client->async_send_request(
        request,
        [self, on_result](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result_future) {
          self->service_pending_ = false;
          try {
            auto response = result_future.get();
            on_result(response->success, response->message);
          } catch (const std::exception &e) {
            on_result(false, std::string("Service call exception: ") + e.what());
          }
        });
  }

  template <typename... Args> void msg_(const char *fmt, Args... args) {
    // Clear current line and print message
    std::printf("\r\033[K");
    if constexpr (sizeof...(Args) == 0) {
      std::printf("%s", fmt);
    } else {
      std::printf(fmt, args...);
    }
    std::printf("\n");
    std::fflush(stdout);
  }
};
} // namespace episode_recorder

int main(int argc, char **argv) {
  TerminalRawMode terminal;
  if (!terminal.enable()) {
    std::fprintf(stderr, "Failed to enable raw terminal mode. "
                         "Are you running in a terminal?\n");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<episode_recorder::TeleopEpisodeKeyboard>(terminal);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}