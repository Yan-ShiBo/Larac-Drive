#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
class CbfRoverStatePlugin final : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    initialize_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CbfRoverStatePlugin::InitializeTransport, this)
    );
  }

private:
  void InitializeTransport()
  {
    if (node_)
    {
      return;
    }
    node_.reset(new transport::Node());
    node_->Init(world_->Name());
    publisher_ = node_->Advertise<msgs::PosesStamped>(
      "~/" + model_->GetName() + "/cbf_state",
      1
    );
    request_subscriber_ = node_->Subscribe(
      "~/" + model_->GetName() + "/cbf_state/request",
      &CbfRoverStatePlugin::OnStateRequest,
      this
    );
  }

  void OnStateRequest(const ::ConstIntPtr &)
  {
    PublishState();
  }

  void PublishState()
  {
    const auto iterations = world_->Iterations();
    if (iterations > std::numeric_limits<std::uint32_t>::max())
    {
      throw std::runtime_error(
        "Gazebo iteration count exceeds timestamped-state message capacity"
      );
    }
    msgs::PosesStamped message;
    msgs::Set(message.mutable_time(), world_->SimTime());
    auto *pose = message.add_pose();
    pose->set_name(model_->GetName());
    pose->set_id(static_cast<std::uint32_t>(iterations));
    msgs::Set(pose, model_->WorldPose());
    publisher_->Publish(message);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  transport::NodePtr node_;
  transport::PublisherPtr publisher_;
  transport::SubscriberPtr request_subscriber_;
  event::ConnectionPtr initialize_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(CbfRoverStatePlugin)
}  // namespace gazebo
