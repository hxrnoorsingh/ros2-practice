#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Color.hh>
#include <gazebo/msgs/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include "explainable_robot/msg/robot_state.hpp"
#include <thread>

namespace gazebo
{
  class StateVisualizerPlugin : public ModelPlugin
  {
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      if (!rclcpp::ok()) rclcpp::init(0, nullptr);
      this->ros_node = rclcpp::Node::make_shared("gazebo_state_visualizer");

      this->state_sub = this->ros_node->create_subscription<explainable_robot::msg::RobotState>(
        "/robot_state", 10, std::bind(&StateVisualizerPlugin::OnState, this, std::placeholders::_1));

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->Name());
      this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
      this->requestPub = this->node->Advertise<msgs::Request>("~/request");

      this->is_alert_visible = false;

      RCLCPP_INFO(this->ros_node->get_logger(), "StateVisualizer Factory Plugin Loaded");
      this->thread = std::thread([this]() { rclcpp::spin(this->ros_node); });
    }

    private: void OnState(const explainable_robot::msg::RobotState::SharedPtr _msg)
    {
        bool should_be_visible = (_msg->state == "ERROR");

        if (should_be_visible && !this->is_alert_visible)
        {
            SpawnAlertSphere();
            this->is_alert_visible = true;
        }
        else if (!should_be_visible && this->is_alert_visible)
        {
            DeleteAlertSphere();
            this->is_alert_visible = false;
        }
    }

    private: void SpawnAlertSphere()
    {
        msgs::Factory msg;
        auto pose = this->model->WorldPose();
        
        std::stringstream sdf;
        sdf << "<?xml version='1.0'?>"
            << "<sdf version='1.6'>"
            << "<model name='alert_sphere'>"
            << "  <pose>" << pose.Pos().X() << " " << pose.Pos().Y() << " " << pose.Pos().Z() + 1.0 << " 0 0 0</pose>"
            << "  <link name='link'>"
            << "    <visual name='visual'>"
            << "      <geometry><sphere><radius>0.3</radius></sphere></geometry>"
            << "      <material><script><name>Gazebo/Red</name></script></material>"
            << "    </visual>"
            << "  </link>"
            << "</model>"
            << "</sdf>";
        
        msg.set_sdf(sdf.str());
        this->factoryPub->Publish(msg);
        RCLCPP_INFO(this->ros_node->get_logger(), "SPAWNED ALERT SPHERE");
    }

    private: void DeleteAlertSphere()
    {
        msgs::Request *msg = msgs::CreateRequest("entity_delete", "alert_sphere");
        this->requestPub->Publish(*msg);
        delete msg;
        RCLCPP_INFO(this->ros_node->get_logger(), "DELETED ALERT SPHERE");
    }

    private: physics::ModelPtr model;
    private: transport::NodePtr node;
    private: transport::PublisherPtr factoryPub;
    private: transport::PublisherPtr requestPub;
    private: rclcpp::Node::SharedPtr ros_node;
    private: rclcpp::Subscription<explainable_robot::msg::RobotState>::SharedPtr state_sub;
    private: std::thread thread;
    private: bool is_alert_visible;
  };
  GZ_REGISTER_MODEL_PLUGIN(StateVisualizerPlugin)
}
