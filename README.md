Generic control toolbox  [![Build Status](https://travis-ci.org/diogoalmeida/generic_control_toolbox.svg?branch=master)](https://travis-ci.org/diogoalmeida/generic_control_toolbox)
========
This ROS package defines libraries that are generic to the control algorithms
used in my research, in an attempt to maximize code re-usability.

[ROS-wiki page](https://wiki.ros.org/generic_control_toolbox).

[Tutorials](https://wiki.ros.org/generic_control_toolbox/Tutorials)

## Dependencies

This is a ROS package and relies on a ROS instalation. Assuming the "full" version of your ROS distro, this package depends on the package ``realtime_tools``:
```
  $ sudo apt-get install ros-<distro>-realtime-tools
```
where you should replace ``<distro>`` with the ROS distribution name (e.g., ``indigo``).

Tested in ROS indigo and kinetic.

## Implementing a controller

To implement a controller you inherit from the ``ControllerTemplate`` class and implement the pure virtual methods. This will enhance your controller with an actionlib interface.

#### Example
Create a package named ``my_controller``, which should define an actionlib action file (``Example.action``) as in the [actionlib tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29). In the class definition file, you should have something like

```cpp
// Other includes
#include <my_controller/ExampleAction.h>
#include <generic_control_toolbox/controller_template.hpp>

class MyController : public generic_control_toolbox::ControllerTemplate<ExampleAction,
                                                                        ExampleGoal,
                                                                        ExampleFeedback,
                                                                        ExampleResult>
{
public:
  MyController(const std::string &action_name);

  // destructor, other public methods/members

private:
  sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);
  bool parseGoal(boost::shared_ptr<const ExampleGoal> goal);
  bool init();
  void resetController();

  // other public methods/members
};
```

The ``action_name`` element of the constructor must be passed to the ``ControllerTemplate`` constructor to initialize the actiolib server. An example of an implemented controller using this template can be found in the sarafun_folding_assembly [folding controller](https://github.com/diogoalmeida/sarafun_folding_assembly/blob/e86eb85feb5480039139a14034bf70dd68f10991/include/folding_assembly_controller/folding_controller.hpp) definition.
