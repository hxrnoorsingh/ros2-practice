#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include "explainable_robot/msg/robot_state.hpp"
#include "explainable_robot/msg/uncertainty.hpp"

namespace explainable_robot
{
  class StateVisualizerPlugin :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure
  {
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override
    {
      this->model = ignition::gazebo::Model(_entity);
      if (!this->model.Valid(_ecm))
      {
        ignerr << "StateVisualizerPlugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
      }

      // Initialize ROS 2
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }
      this->ros_node = rclcpp::Node::make_shared("gazebo_state_visualizer");

      this->state_sub = this->ros_node->create_subscription<explainable_robot::msg::RobotState>(
        "/robot_state", 10, std::bind(&StateVisualizerPlugin::OnState, this, std::placeholders::_1));

      this->uncertainty_sub = this->ros_node->create_subscription<explainable_robot::msg::Uncertainty>(
        "/uncertainty", 10, std::bind(&StateVisualizerPlugin::OnUncertainty, this, std::placeholders::_1));

      RCLCPP_INFO(this->ros_node->get_logger(), "StateVisualizerPlugin configured for model: %s", this->model.Name(_ecm).c_str());
      
      // Spinner for ROS 2
      this->thread = std::thread([this]() {
        rclcpp::spin(this->ros_node);
      });
    }

    public: ~StateVisualizerPlugin()
    {
      if (this->thread.joinable())
      {
        rclcpp::shutdown();
        this->thread.join();
      }
    }

    private: void OnState(const explainable_robot::msg::RobotState::SharedPtr _msg)
    {
        ignmsg << "Robot State Updated: " << _msg->state << std::endl;
    }

    private: void OnUncertainty(const explainable_robot::msg::Uncertainty::SharedPtr _msg)
    {
        // uncertainty logic
    }

    private: ignition::gazebo::Model model;
    private: rclcpp::Node::SharedPtr ros_node;
    private: rclcpp::Subscription<explainable_robot::msg::RobotState>::SharedPtr state_sub;
    private: rclcpp::Subscription<explainable_robot::msg::Uncertainty>::SharedPtr uncertainty_sub;
    private: std::thread thread;
  };
}

IGNITION_ADD_PLUGIN(
    explainable_robot::StateVisualizerPlugin,
    ignition::gazebo::System,
    explainable_robot::StateVisualizerPlugin::ISystemConfigure)
