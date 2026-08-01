#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>

namespace
{
constexpr char kProtocol[] = "GZCB1";
constexpr auto kStepTimeout = std::chrono::seconds(15);
constexpr auto kStatePollInterval = std::chrono::milliseconds(5);
using SteadyClock = std::chrono::steady_clock;

bool EnvironmentFlagEnabled(const char *_name)
{
  const char *value = std::getenv(_name);
  return value != nullptr
    && std::string(value) != ""
    && std::string(value) != "0"
    && std::string(value) != "false"
    && std::string(value) != "FALSE";
}

double EnvironmentPositiveSeconds(const char *_name, double _fallback)
{
  const char *value = std::getenv(_name);
  if (value == nullptr || std::string(value).empty())
  {
    return _fallback;
  }
  const double seconds = std::stod(value);
  if (!std::isfinite(seconds) || seconds <= 0.0)
  {
    throw std::runtime_error(
      std::string(_name) + " must be finite and positive"
    );
  }
  return seconds;
}

double DurationMilliseconds(const SteadyClock::duration &_duration)
{
  return std::chrono::duration<double, std::milli>(_duration).count();
}

double TimeSeconds(const gazebo::msgs::Time &_time)
{
  return static_cast<double>(_time.sec())
    + static_cast<double>(_time.nsec()) * 1.0e-9;
}

struct BridgeState
{
  std::mutex mutex;
  std::condition_variable condition;
  bool pose_seen{false};
  bool stats_seen{false};
  ignition::math::Pose3d pose;
  double pose_sim_time_s{0.0};
  std::uint64_t pose_iterations{0};
  double sim_time_s{0.0};
  std::uint64_t iterations{0};
  std::uint64_t pose_updates{0};
  std::uint64_t stats_updates{0};
};

class GazeboClassicBridge
{
public:
  GazeboClassicBridge(std::string world_name, std::string model_name)
      : world_name_(std::move(world_name)),
        model_name_(std::move(model_name)),
        timing_enabled_(
          EnvironmentFlagEnabled("CBF_ROVER_GAZEBO_BRIDGE_TIMING")
        ),
        startup_timeout_s_(
          EnvironmentPositiveSeconds(
            "CBF_ROVER_GAZEBO_STARTUP_TIMEOUT_S",
            30.0
          )
        )
  {
  }

  void Connect()
  {
    if (!gazebo::transport::waitForNamespaces(
          gazebo::common::Time(startup_timeout_s_)
        ))
    {
      throw std::runtime_error(
        "timed out waiting for the Gazebo world namespace"
      );
    }

    node_.reset(new gazebo::transport::Node());
    node_->Init(world_name_);
    pose_subscriber_ = node_->Subscribe(
      "~/" + model_name_ + "/cbf_state",
      &GazeboClassicBridge::OnPose,
      this
    );
    stats_subscriber_ = node_->Subscribe(
      "~/world_stats",
      &GazeboClassicBridge::OnStats,
      this
    );
    velocity_publisher_ =
      node_->Advertise<gazebo::msgs::Pose>("~/" + model_name_ + "/vel_cmd");
    control_publisher_ =
      node_->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    state_request_publisher_ =
      node_->Advertise<gazebo::msgs::Int>(
        "~/" + model_name_ + "/cbf_state/request"
      );

    const gazebo::common::Time connection_timeout(startup_timeout_s_);
    if (!velocity_publisher_->WaitForConnection(connection_timeout))
    {
      throw std::runtime_error(
        "timed out waiting for the differential-drive command topic"
      );
    }
    if (!control_publisher_->WaitForConnection(connection_timeout))
    {
      throw std::runtime_error(
        "timed out waiting for the world-control topic"
      );
    }
    gazebo::msgs::WorldControl bootstrap;
    bootstrap.set_pause(true);
    bootstrap.set_multi_step(1);
    control_publisher_->Publish(bootstrap, true);

    if (!state_request_publisher_->WaitForConnection(connection_timeout))
    {
      throw std::runtime_error(
        "timed out waiting for the timestamped-state request topic"
      );
    }

    const auto deadline = std::chrono::steady_clock::now()
      + std::chrono::duration<double>(startup_timeout_s_);
    std::unique_lock<std::mutex> lock(state_.mutex);
    const auto ready = [this] {
      return state_.pose_seen
        && state_.pose_iterations >= 1
        && state_.pose_sim_time_s > 0.0;
    };
    while (!ready() && std::chrono::steady_clock::now() < deadline)
    {
      lock.unlock();
      RequestState();
      lock.lock();
      state_.condition.wait_for(lock, std::chrono::milliseconds(100), ready);
    }
    if (!ready())
    {
      std::cerr << "startup synchronization timeout: iterations="
                << state_.iterations
                << " stats_time=" << std::setprecision(17)
                << state_.sim_time_s
                << " pose_time=" << state_.pose_sim_time_s
                << " pose_iterations=" << state_.pose_iterations
                << " pose_seen=" << state_.pose_seen
                << " stats_seen=" << state_.stats_seen
                << " pose_updates=" << state_.pose_updates
                << " stats_updates=" << state_.stats_updates << '\n';
      throw std::runtime_error(
        "timed out waiting for the bootstrap timestamped pose"
      );
    }
  }

  void EmitReady() const
  {
    std::cout << "READY " << kProtocol << ' ' << world_name_ << ' '
              << model_name_ << '\n';
    std::cout.flush();
  }

  void EmitState()
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    const auto rotation = state_.pose.Rot().Euler();
    std::cout << std::setprecision(17)
              << "STATE " << state_.pose_sim_time_s << ' '
              << state_.pose_iterations
              << ' ' << state_.pose.Pos().X()
              << ' ' << state_.pose.Pos().Y()
              << ' ' << state_.pose.Pos().Z()
              << ' ' << rotation.X()
              << ' ' << rotation.Y()
              << ' ' << rotation.Z() << '\n';
    std::cout.flush();
  }

  void Step(double linear_mps, double angular_radps, std::uint32_t steps)
  {
    const auto step_start = SteadyClock::now();
    if (!std::isfinite(linear_mps) || !std::isfinite(angular_radps))
    {
      throw std::runtime_error("STEP command values must be finite");
    }
    if (steps == 0)
    {
      throw std::runtime_error("STEP physics iteration count must be positive");
    }

    gazebo::msgs::Pose velocity;
    gazebo::msgs::Set(
      velocity.mutable_position(),
      ignition::math::Vector3d(linear_mps, 0.0, 0.0)
    );
    gazebo::msgs::Set(
      velocity.mutable_orientation(),
      ignition::math::Quaterniond(0.0, 0.0, angular_radps)
    );
    const auto velocity_publish_start = SteadyClock::now();
    velocity_publisher_->Publish(velocity, true);
    const auto velocity_publish_end = SteadyClock::now();

    std::uint64_t target_iterations;
    std::uint64_t stats_updates_before;
    std::uint64_t pose_updates_before;
    {
      std::lock_guard<std::mutex> lock(state_.mutex);
      target_iterations = state_.pose_iterations + steps;
      stats_updates_before = state_.stats_updates;
      pose_updates_before = state_.pose_updates;
    }

    gazebo::msgs::WorldControl control;
    control.set_pause(true);
    control.set_multi_step(steps);
    const auto control_publish_start = SteadyClock::now();
    control_publisher_->Publish(control, true);
    const auto control_publish_end = SteadyClock::now();

    const auto deadline = SteadyClock::now() + kStepTimeout;
    SteadyClock::duration state_request_publish_duration =
      SteadyClock::duration::zero();
    std::uint64_t state_request_count = 0;
    std::unique_lock<std::mutex> lock(state_.mutex);
    const auto ready = [this, target_iterations] {
      return state_.pose_seen
        && state_.pose_iterations >= target_iterations;
    };
    while (!ready() && SteadyClock::now() < deadline)
    {
      lock.unlock();
      const auto request_start = SteadyClock::now();
      RequestState();
      state_request_publish_duration += SteadyClock::now() - request_start;
      ++state_request_count;
      lock.lock();
      state_.condition.wait_for(lock, kStatePollInterval, ready);
    }
    if (!ready())
    {
      std::cerr << "step synchronization timeout: target_iterations="
                << target_iterations
                << " actual_iterations=" << state_.iterations
                << " stats_time=" << std::setprecision(17)
                << state_.sim_time_s
                << " pose_time=" << state_.pose_sim_time_s
                << " pose_iterations=" << state_.pose_iterations
                << " pose_updates=" << state_.pose_updates
                << " stats_updates=" << state_.stats_updates << '\n';
      throw std::runtime_error("timed out waiting for Gazebo physics step");
    }
    const auto synchronized = SteadyClock::now();
    const auto actual_iterations = state_.pose_iterations;
    const auto stats_updates = state_.stats_updates - stats_updates_before;
    const auto pose_updates = state_.pose_updates - pose_updates_before;
    lock.unlock();
    const auto emit_start = SteadyClock::now();
    EmitState();
    const auto step_end = SteadyClock::now();
    if (timing_enabled_)
    {
      std::cerr << std::fixed << std::setprecision(6)
                << "TIMING step=" << ++step_sequence_
                << " total_ms="
                << DurationMilliseconds(step_end - step_start)
                << " velocity_publish_ms="
                << DurationMilliseconds(
                  velocity_publish_end - velocity_publish_start
                )
                << " control_publish_ms="
                << DurationMilliseconds(
                  control_publish_end - control_publish_start
                )
                << " synchronize_ms="
                << DurationMilliseconds(synchronized - control_publish_end)
                << " state_request_publish_ms="
                << DurationMilliseconds(state_request_publish_duration)
                << " emit_state_ms="
                << DurationMilliseconds(step_end - emit_start)
                << " state_requests=" << state_request_count
                << " stats_updates=" << stats_updates
                << " pose_updates=" << pose_updates
                << " target_iterations=" << target_iterations
                << " actual_iterations=" << actual_iterations << '\n';
    }
  }

  void Stop()
  {
    gazebo::msgs::Pose velocity;
    gazebo::msgs::Set(
      velocity.mutable_position(),
      ignition::math::Vector3d::Zero
    );
    gazebo::msgs::Set(
      velocity.mutable_orientation(),
      ignition::math::Quaterniond::Identity
    );
    velocity_publisher_->Publish(velocity, true);
  }

private:
  void RequestState()
  {
    gazebo::msgs::Int request;
    request.set_data(1);
    state_request_publisher_->Publish(request, true);
  }

  void OnPose(const ::ConstPosesStampedPtr &_message)
  {
    for (int index = 0; index < _message->pose_size(); ++index)
    {
      const auto &pose_message = _message->pose(index);
      if (pose_message.name() != model_name_ || !pose_message.has_id())
      {
        continue;
      }
      std::lock_guard<std::mutex> lock(state_.mutex);
      state_.pose = gazebo::msgs::ConvertIgn(pose_message);
      state_.pose_sim_time_s = TimeSeconds(_message->time());
      state_.pose_iterations = pose_message.id();
      state_.pose_seen = true;
      ++state_.pose_updates;
      state_.condition.notify_all();
      return;
    }
  }

  void OnStats(const ::ConstWorldStatisticsPtr &_message)
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.sim_time_s = TimeSeconds(_message->sim_time());
    state_.iterations = _message->iterations();
    state_.stats_seen = true;
    ++state_.stats_updates;
    state_.condition.notify_all();
  }

  std::string world_name_;
  std::string model_name_;
  BridgeState state_;
  gazebo::transport::NodePtr node_;
  gazebo::transport::SubscriberPtr pose_subscriber_;
  gazebo::transport::SubscriberPtr stats_subscriber_;
  gazebo::transport::PublisherPtr velocity_publisher_;
  gazebo::transport::PublisherPtr control_publisher_;
  gazebo::transport::PublisherPtr state_request_publisher_;
  bool timing_enabled_{false};
  double startup_timeout_s_{30.0};
  std::uint64_t step_sequence_{0};
};

struct Arguments
{
  std::string world_name;
  std::string model_name;
};

Arguments ParseArguments(int argc, char **argv)
{
  Arguments arguments;
  for (int index = 1; index < argc; ++index)
  {
    const std::string value(argv[index]);
    if (value == "--world" && index + 1 < argc)
    {
      arguments.world_name = argv[++index];
    }
    else if (value == "--model" && index + 1 < argc)
    {
      arguments.model_name = argv[++index];
    }
    else
    {
      throw std::runtime_error("usage: bridge --world NAME --model NAME");
    }
  }
  if (arguments.world_name.empty() || arguments.model_name.empty())
  {
    throw std::runtime_error("usage: bridge --world NAME --model NAME");
  }
  return arguments;
}
}  // namespace

int main(int argc, char **argv)
{
  std::locale::global(std::locale::classic());
  std::cin.imbue(std::locale::classic());
  std::cout.imbue(std::locale::classic());

  try
  {
    const auto arguments = ParseArguments(argc, argv);
    gazebo::client::setup(argc, argv);
    GazeboClassicBridge bridge(arguments.world_name, arguments.model_name);
    bridge.Connect();
    bridge.EmitReady();
    bridge.EmitState();

    std::string line;
    while (std::getline(std::cin, line))
    {
      std::istringstream stream(line);
      stream.imbue(std::locale::classic());
      std::string command;
      stream >> command;
      if (command == "GET")
      {
        bridge.EmitState();
      }
      else if (command == "STEP")
      {
        double linear_mps;
        double angular_radps;
        std::uint32_t steps;
        if (!(stream >> linear_mps >> angular_radps >> steps))
        {
          throw std::runtime_error("malformed STEP command");
        }
        std::string trailing;
        if (stream >> trailing)
        {
          throw std::runtime_error("unexpected trailing STEP field");
        }
        bridge.Step(linear_mps, angular_radps, steps);
      }
      else if (command == "STOP")
      {
        bridge.Stop();
        break;
      }
      else if (!command.empty())
      {
        throw std::runtime_error("unknown command: " + command);
      }
    }
    gazebo::client::shutdown();
    return EXIT_SUCCESS;
  }
  catch (const std::exception &error)
  {
    std::cerr << "ERROR " << error.what() << '\n';
    gazebo::client::shutdown();
    return EXIT_FAILURE;
  }
}
