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
// TAO CHU D TREN MAT PHANG X-Y
// ============================================================

std::vector<geometry_msgs::msg::Pose> createLetterD(
    const geometry_msgs::msg::Pose &origin,
    double height,
    double width,
    double direction,
    int arc_points)
{
    std::vector<geometry_msgs::msg::Pose> points;

    // P0
    points.push_back(origin);

    // P0 -> P1 theo truc Y
    auto top = origin;
    top.position.y += height;
    points.push_back(top);

    // Nua ellipse P1 -> P0
    for (int i = 1; i <= arc_points; ++i)
    {
        const double theta =
            PI / 2.0 -
            PI * static_cast<double>(i) /
            static_cast<double>(arc_points);

        auto p = origin;

        p.position.x =
            origin.position.x +
            direction * width * std::cos(theta);

        p.position.y =
            origin.position.y +
            height / 2.0 +
            (height / 2.0) * std::sin(theta);

        points.push_back(p);
    }

    return points;
}


// ============================================================
// RVIZ MARKER
// ============================================================

void publishGuide(
    const rclcpp::Node::SharedPtr &node,
    const rclcpp::Publisher<
        visualization_msgs::msg::Marker>::SharedPtr &pub,
    const std::vector<geometry_msgs::msg::Pose> &points,
    const std::string &frame)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = node->now();

    marker.ns = "letter_d_guide";
    marker.id = 0;

    marker.type =
        visualization_msgs::msg::Marker::LINE_STRIP;

    marker.action =
        visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    for (const auto &p : points)
        marker.points.push_back(p.position);

    pub->publish(marker);
}


// ============================================================
// MAIN
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "draw_letter_d",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    auto logger = node->get_logger();


    // --------------------------------------------------------
    // PARAMETERS
    // --------------------------------------------------------

    const double height =
        node->get_parameter("letter_height").as_double();

    const double width =
        node->get_parameter("letter_width").as_double();

    const int arc_points =
        node->get_parameter("arc_points").as_int();

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

    const int repeat_count =
        node->get_parameter("repeat_count").as_int();

    const double repeat_delay =
        node->get_parameter("repeat_delay").as_double();

    const double wait_time =
        node->get_parameter("guide_wait_time").as_double();

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


    RCLCPP_INFO(
        logger,
        "D: X-Y plane, Z = %.3f m, size %.1f x %.1f cm",
        origin.position.z,
        height * 100.0,
        width * 100.0);


    // --------------------------------------------------------
    // TAO 2 HUONG CHU D
    // --------------------------------------------------------

    const auto positive =
        createLetterD(
            origin,
            height,
            width,
            1.0,
            arc_points);

    const auto negative =
        createLetterD(
            origin,
            height,
            width,
            -1.0,
            arc_points);


    moveit_msgs::msg::RobotTrajectory
        trajectory_positive,
        trajectory_negative;


    move_group.setStartStateToCurrentState();

    const double fraction_positive =
        move_group.computeCartesianPath(
            positive,
            eef_step,
            jump_threshold,
            trajectory_positive,
            true);


    move_group.setStartStateToCurrentState();

    const double fraction_negative =
        move_group.computeCartesianPath(
            negative,
            eef_step,
            jump_threshold,
            trajectory_negative,
            true);


    RCLCPP_INFO(
        logger,
        "D +X: %.2f%% | D -X: %.2f%%",
        fraction_positive * 100.0,
        fraction_negative * 100.0);


    // --------------------------------------------------------
    // CHON HUONG TOT HON
    // --------------------------------------------------------

    const bool use_positive =
        fraction_positive >= fraction_negative;

    const double best_fraction =
        use_positive
            ? fraction_positive
            : fraction_negative;

    const auto &best_trajectory =
        use_positive
            ? trajectory_positive
            : trajectory_negative;

    const auto &best_points =
        use_positive
            ? positive
            : negative;


    RCLCPP_INFO(
        logger,
        "Selected %s, Cartesian path = %.2f%%",
        use_positive ? "+X" : "-X",
        best_fraction * 100.0);


    if (best_fraction < min_fraction)
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
                "letter_d_guide",
                rclcpp::QoS(1).transient_local());


    // Publish lai dinh ky de RViz luon nhan duoc Marker
    auto guide_timer =
        node->create_wall_timer(
            500ms,
            [node, guide_pub, &best_points, frame]()
            {
                publishGuide(
                    node,
                    guide_pub,
                    best_points,
                    frame);
            });

    (void)guide_timer;


    publishGuide(
        node,
        guide_pub,
        best_points,
        frame);


    RCLCPP_INFO(
        logger,
        "Drawing starts in %.1f seconds",
        wait_time);


    std::this_thread::sleep_for(
        std::chrono::duration<double>(
            wait_time));


    // --------------------------------------------------------
    // VE CHU D
    // --------------------------------------------------------

    int count = 1;

    while (
        rclcpp::ok() &&
        (repeat_count == 0 ||
         count <= repeat_count))
    {
        RCLCPP_INFO(
            logger,
            "Drawing D #%d",
            count);


        if (move_group.execute(
                best_trajectory) !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                logger,
                "Trajectory execution failed");

            break;
        }


        RCLCPP_INFO(
            logger,
            "D #%d completed",
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