#ifndef __CONTROLLER_TEMPLATE__
#define __CONTROLLER_TEMPLATE__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>

namespace generic_control_toolbox
{
  /**
  Defines the basic cartesian controller interface.
  **/
  class ControllerBase
  {
  public:
    ControllerBase();
    virtual ~ControllerBase();

    /**
      Method for computing the desired joint states given the control algorithm.

      @param current_state Current joint states.
      @param dt Elapsed time since last control loop.

      @return Desired joint states.
    **/
    virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, const ros::Duration &dt) = 0;
  };

  /**
    A controller interface which implements the SimpleActionServer actionlib
    protocol.
  **/
  template <class ActionClass, class ActionFeedback, class ActionResult>
  class ControllerTemplate : public ControllerBase
  {
  public:
    ControllerTemplate(const std::string action_name);
    virtual ~ControllerTemplate();

  protected:
    boost::shared_ptr<actionlib::SimpleActionServer<ActionClass> > action_server_;
    ActionFeedback feedback_;
    ActionResult result_;
    std::string action_name_;
    ros::NodeHandle nh_;

    /**
      Goal callback method to be implemented in the cartesian controllers.
    **/
    virtual void goalCB() = 0;

    /**
      Preempt callback method to be implemented in the cartesian controllers.
    **/
    virtual void preemptCB() = 0;

    /**
      Method that manages the starting of the actionlib server of each cartesian
    controller.
    **/
    void startActionlib();
  };

  template <class ActionClass, class ActionFeedback, class ActionResult>
  ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::ControllerTemplate(const std::string action_name) : action_name_(action_name)
  {
    nh_ = ros::NodeHandle("~");
    startActionlib();
  }

  template <class ActionClass, class ActionFeedback, class ActionResult>
  ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::~ControllerTemplate() {}

  template <class ActionClass, class ActionFeedback, class ActionResult>
  void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::startActionlib()
  {
    // Initialize actionlib server
    action_server_ = boost::shared_ptr<actionlib::SimpleActionServer<ActionClass> >(new actionlib::SimpleActionServer<ActionClass>(nh_, action_name_, false));

    // Register callbacks
    action_server_->registerGoalCallback(boost::bind(&ControllerTemplate::goalCB, this));
    action_server_->registerPreemptCallback(boost::bind(&ControllerTemplate::preemptCB, this));

    action_server_->start();

    ROS_INFO("%s initialized successfully!", action_name_.c_str());
  }
}

#endif
