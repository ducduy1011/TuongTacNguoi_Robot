#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using namespace std::chrono_literals;

constexpr double PI = 3.14159265358979323846;


// ============================================================
// TAO HINH TRON TREN MAT PHANG X-Y
// ============================================================

std::vector<geometry_msgs::msg::Pose> createCircle(
    const geometry_msgs::msg::Pose &origin,
    double radius,
    int points)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (int i = 0; i <= points; ++i)
    {
        const double theta =
            2.0 * PI * i / points;

        auto pose = origin;

        pose.position.x =
            origin.position.x +
            radius * (std::cos(theta) - 1.0);

        pose.position.y =
            origin.position.y +
            radius * std::sin(theta);

        // Z va orientation giu nguyen
        waypoints.push_back(pose);
    }

    return waypoints;
}


// ============================================================
// RVIZ MARKER
// ============================================================

void publishGuide(
    const rclcpp::Node::SharedPtr &node,
    const rclcpp::Publisher<
        visualization_msgs::msg::Marker>::SharedPtr &pub,
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    const std::string &frame)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = node->now();

    marker.ns = "circle_guide";
    marker.id = 0;

    marker.type =
        visualization_msgs::msg::Marker::LINE_STRIP;

    marker.action =
        visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.008;

    // Mau xanh la
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    for (const auto &pose : waypoints)
        marker.points.push_back(pose.position);

    pub->publish(marker);
}


// ============================================================
// MAIN
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "draw_circle",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    auto logger = node->get_logger();


    // --------------------------------------------------------
    // PARAMETERS
    // --------------------------------------------------------

    const double radius =
        node->get_parameter("circle_radius").as_double();

    const int points =
        node->get_parameter("circle_points").as_int();

    const double eef_step =
        node->get_parameter("eef_step").as_double();

    const double jump_threshold =
        node->get_parameter("jump_threshold").as_double();

    const double min_fraction =
        node->get_parameter("min_fraction").as_double();

    const double velocity =
        node->get_parameter("velocity_scale").as_double();

    const double acceleration =
        node->get_parameter("acceleration_scale").as_double();

    const double wait_time =
        node->get_parameter("guide_wait_time").as_double();

    const int repeat_count =
        node->get_parameter("repeat_count").as_int();

    const double repeat_delay =
        node->get_parameter("repeat_delay").as_double();

    const auto ready =
        node->get_parameter("ready_joints").as_double_array();


    if (ready.size() != 6)
    {
        RCLCPP_ERROR(
            logger,
            "ready_joints must contain 6 values");

        rclcpp::shutdown();
        return 1;
    }


    // --------------------------------------------------------
    // EXECUTOR
    // --------------------------------------------------------

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread(
        [&executor]()
        {
            executor.spin();
        });


    auto shutdown = [&]()
    {
        executor.cancel();

        if (spin_thread.joinable())
            spin_thread.join();

        rclcpp::shutdown();
    };


    // --------------------------------------------------------
    // MOVEIT
    // --------------------------------------------------------

    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "ur_manipulator");

    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);

    move_group.setMaxVelocityScalingFactor(velocity);
    move_group.setMaxAccelerationScalingFactor(acceleration);

    move_group.startStateMonitor(10.0);


    if (!move_group.getCurrentState(10.0))
    {
        RCLCPP_ERROR(
            logger,
            "Cannot receive robot state");

        shutdown();
        return 1;
    }


    // --------------------------------------------------------
    // PRE-DRAW
    // --------------------------------------------------------

    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(ready);

    moveit::planning_interface::
        MoveGroupInterface::Plan ready_plan;


    if (move_group.plan(ready_plan) !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            logger,
            "Cannot plan PRE-DRAW pose");

        shutdown();
        return 1;
    }


    if (move_group.execute(ready_plan) !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            logger,
            "Cannot reach PRE-DRAW pose");

        shutdown();
        return 1;
    }


    std::this_thread::sleep_for(1s);


    // --------------------------------------------------------
    // POSE BAT DAU
    // --------------------------------------------------------

    const auto current_pose =
        move_group.getCurrentPose();

    const auto origin =
        current_pose.pose;

    const std::string frame =
        current_pose.header.frame_id;

    move_group.setPoseReferenceFrame(frame);


    // --------------------------------------------------------
    // TAO HINH TRON
    // --------------------------------------------------------

    const auto waypoints =
        createCircle(
            origin,
            radius,
            points);


    RCLCPP_INFO(
        logger,
        "Circle X-Y | R=%.1f cm | D=%.1f cm | %zu poses",
        radius * 100.0,
        radius * 200.0,
        waypoints.size());


    // --------------------------------------------------------
    // CARTESIAN PATH
    // --------------------------------------------------------

    moveit_msgs::msg::RobotTrajectory trajectory;

    move_group.setStartStateToCurrentState();

    const double fraction =
        move_group.computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory,
            true);


    RCLCPP_INFO(
        logger,
        "Cartesian path: %.2f%%",
        fraction * 100.0);


    if (fraction < min_fraction)
    {
        RCLCPP_ERROR(
            logger,
            "Path rejected. Required >= %.2f%%",
            min_fraction * 100.0);

        shutdown();
        return 1;
    }


    // --------------------------------------------------------
    // RVIZ GUIDE
    // --------------------------------------------------------

    auto guide_pub =
        node->create_publisher<
            visualization_msgs::msg::Marker>(
                "circle_guide",
                rclcpp::QoS(1).transient_local());


    auto guide_timer =
        node->create_wall_timer(
            500ms,
            [node, guide_pub, waypoints, frame]()
            {
                publishGuide(
                    node,
                    guide_pub,
                    waypoints,
                    frame);
            });

    (void)guide_timer;


    publishGuide(
        node,
        guide_pub,
        waypoints,
        frame);


    RCLCPP_INFO(
        logger,
        "Drawing starts in %.1f seconds",
        wait_time);


    std::this_thread::sleep_for(
        std::chrono::duration<double>(
            wait_time));


    // --------------------------------------------------------
    // VE HINH TRON
    // --------------------------------------------------------

    int count = 1;

    while (
        rclcpp::ok() &&
        (repeat_count == 0 ||
         count <= repeat_count))
    {
        RCLCPP_INFO(
            logger,
            "Drawing circle #%d",
            count);


        if (move_group.execute(trajectory) !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                logger,
                "Circle execution failed");

            break;
        }


        RCLCPP_INFO(
            logger,
            "Circle #%d completed",
            count);


        ++count;


        if (repeat_count == 0 ||
            count <= repeat_count)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(
                    repeat_delay));
        }
    }


    shutdown();
    return 0;
}