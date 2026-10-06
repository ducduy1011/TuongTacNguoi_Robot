#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <future>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <Eigen/Geometry>
#include "ur3_llm_control_week3/srv/execute_skill.hpp"
#include "ur3_llm_control_week3/srv/get_scene_state.hpp"
using namespace std::chrono_literals;
using Skill=ur3_llm_control_week3::srv::ExecuteSkill;
using Scene=ur3_llm_control_week3::srv::GetSceneState;
using Trigger=std_srvs::srv::Trigger;
using Traj=control_msgs::action::FollowJointTrajectory;
using Pos=std::array<double,3>;
constexpr const char *OBJECTS[]={"red_cube","yellow_cube","blue_cube","green_cube","purple_cube"};

class Skills {
 rclcpp::Node::SharedPtr n;
 moveit::planning_interface::MoveGroupInterface arm;
 moveit::planning_interface::PlanningSceneInterface scene;
 rclcpp::CallbackGroup::SharedPtr io, serial;
 rclcpp::Client<Scene>::SharedPtr camera;
 rclcpp::Client<Trigger>::SharedPtr attach,release,grasp_status;
 rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning;
 rclcpp_action::Client<Traj>::SharedPtr gripper;
 rclcpp::Service<Skill>::SharedPtr service;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr held_pub;
 YAML::Node config;
 std::string holding;
 double last_stamp=-1;
 bool constrained=false;
 public:
 explicit Skills(rclcpp::Node::SharedPtr node):n(node),arm(node,"ur_manipulator") {
  config=YAML::LoadFile(ament_index_cpp::get_package_share_directory("ur3_llm_control_week3")+"/config/scene.yaml");
  io=n->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  serial=n->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  camera=n->create_client<Scene>("/get_scene_state",rmw_qos_profile_services_default,io);
  attach=n->create_client<Trigger>("/physical_grasp/attach",rmw_qos_profile_services_default,io);
  release=n->create_client<Trigger>("/physical_grasp/release",rmw_qos_profile_services_default,io);
  grasp_status=n->create_client<Trigger>("/physical_grasp/status",rmw_qos_profile_services_default,io);
  get_planning=n->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",rmw_qos_profile_services_default,io);
  gripper=rclcpp_action::create_client<Traj>(n,"/gripper_controller/follow_joint_trajectory",io);
  held_pub=n->create_publisher<std_msgs::msg::String>("/held_object",rclcpp::QoS(1).transient_local());
  // Recover a live grasp when this node restarts while Gazebo and MoveIt stay up.
  std::string recovered;
  if(trigger(grasp_status,&recovered)){
   if(!valid_object(recovered)||scene.getAttachedObjects({recovered}).count(recovered)!=1)
    throw std::runtime_error("Physical grasp and MoveIt attachment disagree");
   holding=recovered;constrained=true;
   RCLCPP_INFO(n->get_logger(),"Recovered held object: %s",holding.c_str());
  }
  publish_held();
  arm.startStateMonitor(3.0);
  arm.setPoseReferenceFrame("base_link");arm.setEndEffectorLink("grasp_tcp");
  arm.setMaxVelocityScalingFactor(.10);arm.setMaxAccelerationScalingFactor(.10);
  arm.setPlanningTime(10);arm.setNumPlanningAttempts(5);arm.setGoalPositionTolerance(.001);arm.setGoalOrientationTolerance(.005);
  std::vector<moveit_msgs::msg::CollisionObject> environment={
   box("work_table",{.3,0,-.025},{1.1,.9,.05}),box("ground",{0,0,-.775},{10,10,.05})};
  int leg=0;
  for(double x:{-.19,.79})for(double y:{-.39,.39})environment.push_back(box("table_leg_"+std::to_string(++leg),{x,y,-.40},{.07,.07,.70}));
  if(!scene.applyCollisionObjects(environment))throw std::runtime_error("planning scene update failed");
  service=n->create_service<Skill>("/execute_skill",[this](const Skill::Request::SharedPtr req,Skill::Response::SharedPtr res){
   try {res->status=execute(req->skill,req->object,req->target);}catch(const std::exception &e){res->status=std::string("FAILED: ")+e.what();}
   res->success=res->status=="SUCCESS"; RCLCPP_INFO(n->get_logger(),"%s(%s,%s): %s",req->skill.c_str(),req->object.c_str(),req->target.c_str(),res->status.c_str());
  },rmw_qos_profile_services_default,serial);
  RCLCPP_INFO(n->get_logger(),"Skill server ready");
 }
 void publish_held(){std_msgs::msg::String s;s.data=holding;held_pub->publish(s);}
 bool valid_object(const std::string &s){return std::find(std::begin(OBJECTS),std::end(OBJECTS),s)!=std::end(OBJECTS);}
 moveit_msgs::msg::CollisionObject box(const std::string &id,Pos p,Pos size){
  moveit_msgs::msg::CollisionObject c;c.header.frame_id="base_link";c.id=id;c.operation=c.ADD;
  shape_msgs::msg::SolidPrimitive b;b.type=b.BOX;b.dimensions={size[0],size[1],size[2]};
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;pose.position.x=p[0];pose.position.y=p[1];pose.position.z=p[2];
  c.primitives={b};c.primitive_poses={pose};return c;
 }
 YAML::Node observe(){
  if(!camera->wait_for_service(3s))throw std::runtime_error("CAMERA_UNAVAILABLE");
  auto deadline=std::chrono::steady_clock::now()+6s;
  while(std::chrono::steady_clock::now()<deadline){
   auto f=camera->async_send_request(std::make_shared<Scene::Request>());
   if(f.wait_for(2s)!=std::future_status::ready)throw std::runtime_error("CAMERA_TIMEOUT");
   auto r=f.get();if(r->success){auto s=YAML::Load(r->scene_json);double stamp=s["stamp"].as<double>();
    if(s["frame_id"].as<std::string>()!="base_link")throw std::runtime_error("INVALID_CAMERA_FRAME");
    if(stamp>last_stamp){last_stamp=stamp;return s;}
   }
   std::this_thread::sleep_for(100ms);
  }throw std::runtime_error("CAMERA_STALE");
 }
 Pos position(const YAML::Node &o){auto a=o["position"].as<std::vector<double>>();if(a.size()!=3)throw std::runtime_error("INVALID_POSITION");for(double x:a)if(!std::isfinite(x))throw std::runtime_error("INVALID_POSITION");return {a[0],a[1],a[2]};}
 void sync_scene(const YAML::Node &s){
  std::vector<moveit_msgs::msg::CollisionObject> cubes;
  for(const auto *name:OBJECTS)if(holding!=name && s["objects"][name]["detected"].as<bool>())cubes.push_back(box(name,position(s["objects"][name]),{.05,.05,.05}));
  if(!cubes.empty()&&!scene.applyCollisionObjects(cubes))throw std::runtime_error("planning scene update failed");
 }
 bool grip(double value){
  if(!gripper->wait_for_action_server(3s))return false;
  Traj::Goal g;g.trajectory.joint_names={"left_finger_joint","right_finger_joint"};trajectory_msgs::msg::JointTrajectoryPoint p;p.positions={value,value};p.time_from_start.sec=2;g.trajectory.points={p};
  auto f=gripper->async_send_goal(g);if(f.wait_for(4s)!=std::future_status::ready)return false;auto handle=f.get();if(!handle)return false;
  auto result=gripper->async_get_result(handle);if(result.wait_for(15s)!=std::future_status::ready){gripper->async_cancel_goal(handle);return false;}
  auto r=result.get();if(r.code!=rclcpp_action::ResultCode::SUCCEEDED||r.result->error_code!=0)return false;
  auto state=arm.getCurrentState(2.0);if(!state)return false;
  for(const auto &name:{"left_finger_joint","right_finger_joint"})if(std::abs(state->getVariablePosition(name)-value)>.004)return false;
  return true;
 }
 bool trigger(rclcpp::Client<Trigger>::SharedPtr c,std::string *message=nullptr){
  if(!c->wait_for_service(1s))return false;auto f=c->async_send_request(std::make_shared<Trigger::Request>());if(f.wait_for(3s)!=std::future_status::ready)return false;auto r=f.get();if(message)*message=r->message;return r->success;
 }
 bool touch(const std::string &object,bool allow){
  if(!get_planning->wait_for_service(3s))return false;
  auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();req->components.components=moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto f=get_planning->async_send_request(req);if(f.wait_for(3s)!=std::future_status::ready)return false;
  auto acm=f.get()->scene.allowed_collision_matrix;
  auto ensure=[&](const std::string &name){auto it=std::find(acm.entry_names.begin(),acm.entry_names.end(),name);if(it!=acm.entry_names.end())return size_t(it-acm.entry_names.begin());
   size_t i=acm.entry_names.size();acm.entry_names.push_back(name);for(auto &row:acm.entry_values)row.enabled.push_back(false);moveit_msgs::msg::AllowedCollisionEntry row;row.enabled.resize(i+1,false);acm.entry_values.push_back(row);return i;};
  auto i=ensure(object);for(const auto &link:{"left_finger_link","right_finger_link"}){auto j=ensure(link);acm.entry_values[i].enabled[j]=allow;acm.entry_values[j].enabled[i]=allow;}
  moveit_msgs::msg::PlanningScene diff;diff.is_diff=true;diff.allowed_collision_matrix=acm;return scene.applyPlanningScene(diff);
 }
 bool reached(const geometry_msgs::msg::Pose &goal){
  auto state=arm.getCurrentState(3.0);if(!state)return false;
  const auto &tcp=state->getGlobalLinkTransform("grasp_tcp");
  Eigen::Quaterniond actual(tcp.rotation()),desired(goal.orientation.w,goal.orientation.x,goal.orientation.y,goal.orientation.z);
  double distance=(tcp.translation()-Eigen::Vector3d(goal.position.x,goal.position.y,goal.position.z)).norm();
  double angle=2*std::acos(std::clamp(std::abs(actual.dot(desired))/(actual.norm()*desired.norm()),0.0,1.0));
  if(distance<=.002&&angle<=.02)return true;
  RCLCPP_ERROR(n->get_logger(),"TCP residual %.1f mm, %.3f rad",distance*1000,angle);return false;
 }
 bool move(Pos p,bool cartesian=false){
  geometry_msgs::msg::Pose goal;goal.position.x=p[0];goal.position.y=p[1];goal.position.z=p[2];goal.orientation.x=1;goal.orientation.w=0;
  auto current=arm.getCurrentState(3.0);if(!current)return false;
  arm.setStartState(*current);
  if(!cartesian){arm.setPoseTarget(goal);moveit::planning_interface::MoveGroupInterface::Plan plan;auto ok=arm.plan(plan);arm.clearPoseTargets();if(ok!=moveit::core::MoveItErrorCode::SUCCESS)return false;
   if(arm.execute(plan)!=moveit::core::MoveItErrorCode::SUCCESS)return false;
  }else{
   moveit_msgs::msg::RobotTrajectory t;double fraction=arm.computeCartesianPath({goal},.003,0.0,t,true);
   for(size_t i=1;i<t.joint_trajectory.points.size();++i)for(size_t j=0;j<t.joint_trajectory.joint_names.size();++j)if(std::abs(t.joint_trajectory.points[i].positions[j]-t.joint_trajectory.points[i-1].positions[j])>.15)return false;
   if(fraction<.999){RCLCPP_ERROR(n->get_logger(),"Cartesian fraction %.3f",fraction);return false;}
   // Cartesian service computes timestamps; scale time and derivatives for slow execution.
   for(auto &point:t.joint_trajectory.points){double sec=rclcpp::Duration(point.time_from_start).seconds()*3;point.time_from_start=rclcpp::Duration::from_seconds(sec);for(auto &v:point.velocities)v/=3;for(auto &a:point.accelerations)a/=9;}
   if(arm.execute(t)!=moveit::core::MoveItErrorCode::SUCCESS)return false;
  }
  return reached(goal);
 }
 bool observation_pose(){
  arm.setStartStateToCurrentState();
  arm.setJointValueTarget(std::vector<double>{-1.5708,-1.5708,1.5708,-1.5708,-1.5708,0});
  return arm.move()==moveit::core::MoveItErrorCode::SUCCESS;
 }
 std::string execute(const std::string &skill,const std::string &object,const std::string &target){
  if(skill=="home"){
   if(!holding.empty())return "FAILED: OBJECT_HELD";
   return observation_pose()?"SUCCESS":"PLANNING_FAILED";
  }
  if(skill!="pick"&&skill!="place")return "INVALID_SKILL";
  if(!valid_object(object))return "INVALID_OBJECT";
  if(skill=="pick"){
   if(!holding.empty())return "FAILED: OBJECT_HELD";
   auto s=observe();if(!s["objects"][object]["detected"].as<bool>())return "OBJECT_NOT_FOUND";sync_scene(s);
   auto p=position(s["objects"][object]);
   if(p[0]<.15||p[0]>.50||std::abs(p[1])>.30)return "FAILED: OUTSIDE_WORKSPACE";
   if(!grip(.015))return "GRASP_FAILED";
   if(!move({p[0],p[1],.22}))return "PLANNING_FAILED";
   if(!touch(object,true))return "FAILED: COLLISION_PERMISSION";
   if(!move(p,true)){touch(object,false);return "PLANNING_FAILED";}
   if(!grip(0)){touch(object,false);return "GRASP_FAILED";}
   std::string selected;
   bool attached=trigger(attach,&selected);
   if(!attached||selected!=object){
    if(attached&&!trigger(release)){holding=selected;constrained=true;publish_held();return "FAILED: WRONG_OBJECT_RELEASE";}
    touch(object,false);return "GRASP_FAILED";
   }
   constrained=true;
   if(!arm.attachObject(object,"grasp_tcp",{"left_finger_link","right_finger_link"})){
    if(trigger(release)){constrained=false;touch(object,false);}
    else {holding=object;publish_held();}
    return "FAILED: SCENE_ATTACH";
   }
   holding=object;publish_held();
   if(!move({p[0],p[1],.22},true))return "PLANNING_FAILED";
   std::string held; if(!trigger(grasp_status,&held)||held!=object)return "GRASP_FAILED";
   last_stamp=n->now().seconds();observe();return "SUCCESS";
  }
  if(holding!=object)return "FAILED: WRONG_HELD_OBJECT";
  if(scene.getAttachedObjects({object}).count(object)!=1)return "FAILED: SCENE_ATTACHMENT_MISSING";
  if(!config["targets"][target])return "INVALID_TARGET";
  auto s=observe();auto occupancy=target.rfind("zone_",0)==0?s["zones"][target]:s["temporary"][target];
  if(occupancy&&occupancy.as<std::string>()=="unknown"){
   // The lifted cube / arm can hide other cubes. Keep the attachment and move
   // out of the camera view, then require a fresh image before deciding.
   RCLCPP_INFO(n->get_logger(),"Target unknown; moving to observation posture with held object");
   sync_scene(s);
   if(!observation_pose())return "PLANNING_FAILED";
   std::string held;
   if(!trigger(grasp_status,&held)||held!=object)return "GRASP_FAILED";
   last_stamp=n->now().seconds();s=observe();
   occupancy=target.rfind("zone_",0)==0?s["zones"][target]:s["temporary"][target];
  }
  if(!occupancy||occupancy.as<std::string>()!="empty")return "FAILED: TARGET_OCCUPIED_OR_UNKNOWN";
  sync_scene(s);auto v=config["targets"][target].as<std::vector<double>>();Pos p{v[0],v[1],v[2]+.003};
  if(!move({p[0],p[1],.22})||!move(p,true))return "PLANNING_FAILED";
  // Release at the start of opening so the fingers do not drag a constrained cube.
  if(constrained&&!trigger(release))return "FAILED: RELEASE_FAILED";
  if(!grip(.015))return "FAILED: GRIPPER_OPEN_FAILED";
  if(!arm.detachObject(object))return "FAILED: SCENE_DETACH";
  holding.clear();publish_held();constrained=false;
  if(!move({p[0],p[1],.22},true))return "PLANNING_FAILED";
  if(!touch(object,false))return "FAILED: COLLISION_PERMISSION";
  // Move the empty gripper away from the target so RGB perception can see the placed cube.
  if(!observation_pose())return "PLANNING_FAILED";
  last_stamp=n->now().seconds();
  auto deadline=std::chrono::steady_clock::now()+8s;
  double residual=-1;
  while(std::chrono::steady_clock::now()<deadline){s=observe();if(s["objects"][object]["detected"].as<bool>()&&s["objects"][object]["location"].as<std::string>()==target){
    auto actual=position(s["objects"][object]);residual=std::hypot(actual[0]-v[0],actual[1]-v[1]);
    if(residual<=.0025){RCLCPP_INFO(n->get_logger(),"%s center residual %.1f mm",target.c_str(),residual*1000);sync_scene(s);return "SUCCESS";}
   }std::this_thread::sleep_for(200ms);}
  if(residual>=0)RCLCPP_INFO(n->get_logger(),"%s center residual %.1f mm (limit 2.5)",target.c_str(),residual*1000);
  return residual>=0?"FAILED: PLACE_CENTER_ERROR":"FAILED: PLACE_NOT_CONFIRMED_BY_CAMERA";
 }
};
int main(int argc,char **argv){
 rclcpp::init(argc,argv);auto n=std::make_shared<rclcpp::Node>("skill_server",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(),4);ex.add_node(n);std::thread t([&]{ex.spin();});
 try{Skills skills(n);t.join();}catch(const std::exception &e){RCLCPP_FATAL(n->get_logger(),"%s",e.what());ex.cancel();t.join();}
 rclcpp::shutdown();return 0;
}
