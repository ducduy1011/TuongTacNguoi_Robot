#include <array>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ur3_llm_control/srv/execute_skill.hpp"

using namespace std::chrono_literals;

using ExecuteSkill = ur3_llm_control::srv::ExecuteSkill;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;
using Pos = std::array<double, 3>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // NODE
  auto node = std::make_shared<rclcpp::Node>(
    "skill_server",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  auto skill_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto io_group = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);

  executor.add_node(node);

  std::thread spinner([&]() {
    executor.spin();
  });

  // MOVEIT
  moveit::planning_interface::MoveGroupInterface arm(
    node, "ur_manipulator");

  moveit::planning_interface::PlanningSceneInterface scene;

  arm.setPoseReferenceFrame("base_link");
  arm.setEndEffectorLink("tool0");
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(5);
  arm.allowReplanning(true);
  arm.setMaxVelocityScalingFactor(0.10);
  arm.setMaxAccelerationScalingFactor(0.10);
  arm.setGoalPositionTolerance(0.01);
  arm.setGoalOrientationTolerance(0.10);

  // TF
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // GAZEBO
  auto gazebo_client = node->create_client<SetEntityPose>(
    "/world/ur3_task/set_pose",
    rmw_qos_profile_services_default,
    io_group);

  std::map<std::string, Pos> initial_objects = {
    {"red_cube",    {0.35, -0.20, 0.025}},
    {"yellow_cube", {0.35,  0.00, 0.025}},
    {"blue_cube",   {0.35,  0.20, 0.025}}
  };

  auto objects = initial_objects;

  std::map<std::string, Pos> zones = {
    {"zone_a", {0.43,  0.20, 0.025}},
    {"zone_b", {0.43,  0.00, 0.025}},
    {"zone_c", {0.43, -0.20, 0.025}}
  };

  std::string holding;
  std::mutex holding_mutex;

  constexpr double SAFE_Z = 0.28;
  constexpr double ABOVE_Z = 0.12;
  constexpr double GRASP_Z = 0.0575;
  constexpr double CUBE_CENTER_Z = 0.025;

  constexpr double SIDE_OFFSET = 0.06;
  constexpr double OCCUPIED_DISTANCE = 0.055;

  constexpr double CUBE_TOOL_OFFSET_Z =
    GRASP_Z - CUBE_CENTER_Z;

  // GAZEBO POSE REQUEST
  auto make_pose_request =
    [](const std::string &name, const Pos &p)
  {
    auto req = std::make_shared<SetEntityPose::Request>();

    req->entity.name = name;
    req->entity.type = 2;

    req->pose.position.x = p[0];
    req->pose.position.y = p[1];
    req->pose.position.z = p[2];

    req->pose.orientation.w = 1.0;

    return req;
  };

  auto set_gazebo_pose =
    [&](const std::string &name, const Pos &p) -> bool
  {
    if (!gazebo_client->wait_for_service(2s))
      return false;

    auto future =
      gazebo_client->async_send_request(
        make_pose_request(name, p));

    if (future.wait_for(2s) != std::future_status::ready)
      return false;

    return future.get()->success;
  };

  auto set_gazebo_pose_async =
    [&](const std::string &name, const Pos &p)
  {
    if (gazebo_client->service_is_ready())
      gazebo_client->async_send_request(
        make_pose_request(name, p));
  };

  // FOLLOW CUBE
  auto follow_timer = node->create_wall_timer(
    50ms,
    [&]()
    {
      std::string object;

      {
        std::lock_guard<std::mutex> lock(holding_mutex);
        object = holding;
      }

      if (object.empty())
        return;

      try
      {
        auto t = tf_buffer.lookupTransform(
          "base_link",
          "tool0",
          tf2::TimePointZero);

        Pos p = {
          t.transform.translation.x,
          t.transform.translation.y,
          t.transform.translation.z - CUBE_TOOL_OFFSET_Z
        };

        set_gazebo_pose_async(object, p);
      }
      catch (const tf2::TransformException &e)
      {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          2000,
          "%s",
          e.what());
      }
    },
    io_group);

  // TABLE
  auto add_table = [&]()
  {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "work_table";

    auto add_box = [&](const std::array<double, 6> &b)
    {
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {b[0], b[1], b[2]};

      geometry_msgs::msg::Pose pose;
      pose.position.x = b[3];
      pose.position.y = b[4];
      pose.position.z = b[5];
      pose.orientation.w = 1.0;

      table.primitives.push_back(box);
      table.primitive_poses.push_back(pose);
    };

    const std::vector<std::array<double, 6>> parts = {
      {0.63, 0.80, 0.05,  0.435,  0.00, -0.030},
      {0.03, 0.80, 0.05, -0.135,  0.00, -0.030},
      {0.24, 0.28, 0.05,  0.000, -0.26, -0.030},
      {0.24, 0.28, 0.05,  0.000,  0.26, -0.030},

      {0.07, 0.07, 0.70, -0.09, -0.34, -0.40},
      {0.07, 0.07, 0.70, -0.09,  0.34, -0.40},
      {0.07, 0.07, 0.70,  0.69, -0.34, -0.40},
      {0.07, 0.07, 0.70,  0.69,  0.34, -0.40}
    };

    for (const auto &part : parts)
      add_box(part);

    table.operation =
      moveit_msgs::msg::CollisionObject::ADD;

    scene.applyCollisionObject(table);
  };

  // CUBE COLLISION
  auto add_cube =
    [&](const std::string &name, const Pos &p)
  {
    moveit_msgs::msg::CollisionObject object;

    object.header.frame_id = "base_link";
    object.id = name;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.05, 0.05, 0.05};

    geometry_msgs::msg::Pose pose;
    pose.position.x = p[0];
    pose.position.y = p[1];
    pose.position.z = p[2];
    pose.orientation.w = 1.0;

    object.primitives.push_back(box);
    object.primitive_poses.push_back(pose);
    object.operation =
      moveit_msgs::msg::CollisionObject::ADD;

    scene.applyCollisionObject(object);
  };

  // CARTESIAN MOVE
  auto move_cartesian =
    [&](double x, double y, double z) -> bool
  {
    geometry_msgs::msg::Pose target;

    target.position.x = x;
    target.position.y = y;
    target.position.z = z;

    target.orientation.x = 1.0;
    target.orientation.w = 0.0;

    moveit_msgs::msg::RobotTrajectory trajectory;

    arm.setStartStateToCurrentState();

    double fraction = arm.computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{target},
      0.005,
      0.0,
      trajectory,
      true);

    RCLCPP_INFO(
      node->get_logger(),
      "Cartesian x=%.2f y=%.2f z=%.4f : %.1f%%",
      x, y, z, fraction * 100.0);

    if (fraction < 0.99)
      return false;

    moveit::planning_interface::
      MoveGroupInterface::Plan plan;

    plan.trajectory_ = trajectory;

    if (arm.execute(plan) !=
        moveit::core::MoveItErrorCode::SUCCESS)
      return false;

    rclcpp::sleep_for(300ms);
    return true;
  };

  // HOME
  auto home = [&]() -> bool
  {
    const std::vector<double> joints = {
      0.0,
      -1.570085,
      1.570022,
      -1.569948,
      -1.570006,
      0.0
    };

    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);

    bool ok =
      arm.move() ==
      moveit::core::MoveItErrorCode::SUCCESS;

    if (ok)
    {
      rclcpp::sleep_for(500ms);
      RCLCPP_INFO(node->get_logger(), "HOME SUCCESS");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "HOME FAILED");
    }

    return ok;
  };

  // OCCUPANCY
  auto occupied =
    [&](const Pos &p,
        const std::string &ignore) -> bool
  {
    for (const auto &[name, q] : objects)
    {
      if (name == ignore)
        continue;

      double dx = q[0] - p[0];
      double dy = q[1] - p[1];

      if (dx * dx + dy * dy <
          OCCUPIED_DISTANCE * OCCUPIED_DISTANCE)
        return true;
    }

    return false;
  };

  // RESET
  auto reset_system = [&]() -> bool
  {
    for (const auto &[name, p] : initial_objects)
      arm.detachObject(name);

    scene.removeCollisionObjects({
      "work_table",
      "red_cube",
      "yellow_cube",
      "blue_cube"
    });

    {
      std::lock_guard<std::mutex> lock(holding_mutex);
      holding.clear();
    }

    objects = initial_objects;

    for (const auto &[name, p] : objects)
      if (!set_gazebo_pose(name, p))
        return false;

    add_table();

    for (const auto &[name, p] : objects)
      add_cube(name, p);

    rclcpp::sleep_for(1s);

    return home();
  };

  // PICK
  auto pick =
    [&](const std::string &object) -> std::string
  {
    if (!objects.count(object))
      return "INVALID_OBJECT";

    {
      std::lock_guard<std::mutex> lock(holding_mutex);

      if (!holding.empty())
        return "FAILED";
    }

    Pos p = objects.at(object);

    RCLCPP_INFO(
      node->get_logger(),
      "PICK %s",
      object.c_str());

    if (!move_cartesian(p[0], p[1], SAFE_Z))
      return "PLANNING_FAILED";

    if (!move_cartesian(p[0], p[1], ABOVE_Z))
      return "PLANNING_FAILED";

    scene.removeCollisionObjects({object});
    rclcpp::sleep_for(300ms);

    if (!move_cartesian(p[0], p[1], GRASP_Z))
    {
      add_cube(object, p);
      return "PLANNING_FAILED";
    }

    add_cube(object, p);
    rclcpp::sleep_for(300ms);

    if (!arm.attachObject(
          object,
          "tool0",
          {"tool0", "flange", "wrist_3_link"}))
      return "FAILED";

    {
      std::lock_guard<std::mutex> lock(holding_mutex);
      holding = object;
    }

    rclcpp::sleep_for(200ms);

    if (!move_cartesian(p[0], p[1], SAFE_Z))
      return "PLANNING_FAILED";

    RCLCPP_INFO(
      node->get_logger(),
      "PICK SUCCESS");

    return "SUCCESS";
  };

  // PLACE
  auto place =
    [&](const std::string &object,
        const std::string &zone) -> std::string
  {
    if (!objects.count(object))
      return "INVALID_OBJECT";

    if (!zones.count(zone))
      return "INVALID_ZONE";

    {
      std::lock_guard<std::mutex> lock(holding_mutex);

      if (holding != object)
        return "FAILED";
    }

    Pos p = zones.at(zone);

    if (occupied(p, object))
    {
      Pos left  = {p[0], p[1] - SIDE_OFFSET, p[2]};
      Pos right = {p[0], p[1] + SIDE_OFFSET, p[2]};

      if (!occupied(left, object))
      {
        p = left;

        RCLCPP_INFO(
          node->get_logger(),
          "%s occupied -> LEFT",
          zone.c_str());
      }
      else if (!occupied(right, object))
      {
        p = right;

        RCLCPP_INFO(
          node->get_logger(),
          "%s occupied -> RIGHT",
          zone.c_str());
      }
      else
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "%s has no free position",
          zone.c_str());

        return "FAILED";
      }
    }

    RCLCPP_INFO(
      node->get_logger(),
      "PLACE %s -> %s",
      object.c_str(),
      zone.c_str());

    if (!move_cartesian(p[0], p[1], SAFE_Z))
      return "PLANNING_FAILED";

    if (!move_cartesian(p[0], p[1], ABOVE_Z))
      return "PLANNING_FAILED";

    if (!move_cartesian(p[0], p[1], GRASP_Z))
      return "PLANNING_FAILED";

    if (!arm.detachObject(object))
      return "FAILED";

    {
      std::lock_guard<std::mutex> lock(holding_mutex);
      holding.clear();
    }

    if (!set_gazebo_pose(object, p))
      return "FAILED";

    objects[object] = p;

    scene.removeCollisionObjects({object});
    rclcpp::sleep_for(300ms);

    if (!move_cartesian(p[0], p[1], SAFE_Z))
    {
      add_cube(object, p);
      return "PLANNING_FAILED";
    }

    add_cube(object, p);

    RCLCPP_INFO(
      node->get_logger(),
      "PLACE SUCCESS");

    return "SUCCESS";
  };

  // RESET
  if (!reset_system())
  {
    RCLCPP_FATAL(
      node->get_logger(),
      "Initial reset failed");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // SERVICE
  auto service = node->create_service<ExecuteSkill>(
    "/execute_skill",

    [&](const ExecuteSkill::Request::SharedPtr req,
        ExecuteSkill::Response::SharedPtr res)
    {
      if (req->skill == "pick")
        res->status = pick(req->object);

      else if (req->skill == "place")
        res->status =
          place(req->object, req->zone);

      else if (req->skill == "home")
        res->status =
          home() ? "SUCCESS" : "PLANNING_FAILED";

      else
        res->status = "INVALID_SKILL";

      res->success =
        res->status == "SUCCESS";

      RCLCPP_INFO(
        node->get_logger(),
        "%s -> %s",
        req->skill.c_str(),
        res->status.c_str());
    },

    rmw_qos_profile_services_default,
    skill_group);

  RCLCPP_INFO(
    node->get_logger(),
    "Skill server ready.");

  spinner.join();

  rclcpp::shutdown();
  return 0;
}
