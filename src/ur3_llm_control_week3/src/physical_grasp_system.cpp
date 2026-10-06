#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <dlfcn.h>
namespace sim=ignition::gazebo;
using Trigger=std_srvs::srv::Trigger;

// All entity mutations occur in PreUpdate, on the simulator thread.
// This plugin never sets a model/link pose or velocity.
class PhysicalGrasp : public sim::System, public sim::ISystemConfigure, public sim::ISystemPreUpdate {
 sim::Entity robot=sim::kNullEntity, palm=sim::kNullEntity, joint=sim::kNullEntity, child=sim::kNullEntity;
 sim::EntityComponentManager *ecm=nullptr;
 rclcpp::Node::SharedPtr node;
 rclcpp::executors::SingleThreadedExecutor executor;
 rclcpp::Service<Trigger>::SharedPtr attach_service,release_service,status_service;
 std::string holding;
 bool controllers_pinned=false;
 public:
 void Configure(const sim::Entity &entity,const std::shared_ptr<const sdf::Element> &,sim::EntityComponentManager &,sim::EventManager &) override {
  robot=entity;
  if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);}
  node=std::make_shared<rclcpp::Node>("physical_grasp_system");executor.add_node(node);
  attach_service=node->create_service<Trigger>("/physical_grasp/attach",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){
   if(!ecm||!holding.empty()){r->success=false;r->message="NOT_READY_OR_ALREADY_HOLDING";return;}
   sim::Model model(robot);palm=model.LinkByName(*ecm,"gripper_base_link");
   if(palm==sim::kNullEntity){r->message="GRIPPER_LINK_MISSING";return;}
   for(const auto &name:{"left_finger_joint","right_finger_joint"}){
    auto j=model.JointByName(*ecm,name);auto p=ecm->Component<sim::components::JointPosition>(j);
    if(!p||p->Data().empty()||p->Data()[0]>.004){r->message="FINGERS_NOT_CLOSED";return;}
   }
   auto palm_pose=sim::worldPose(palm,*ecm);
   for(const auto &name:{"red_cube","yellow_cube","blue_cube","green_cube","purple_cube"}){
    auto entity=ecm->EntityByComponents(sim::components::Model(),sim::components::Name(name));
    if(entity==sim::kNullEntity)continue;
    auto link=sim::Model(entity).LinkByName(*ecm,"link");if(link==sim::kNullEntity)continue;
    auto relative=palm_pose.Inverse()*sim::worldPose(link,*ecm);
    auto p=relative.Pos();
    if(std::abs(p.X())>.008||std::abs(p.Y())>.012||std::abs(p.Z()-.06)>.012)continue;
    joint=ecm->CreateEntity();child=link;
    ecm->CreateComponent(joint,sim::components::DetachableJoint({palm,child,"fixed"}));
    holding=name;r->success=true;r->message=holding;
    RCLCPP_INFO(node->get_logger(),"PHYSICAL ATTACH %s",name);return;
   }
   r->success=false;r->message="NO_CUBE_BETWEEN_FINGERS";
  });
  release_service=node->create_service<Trigger>("/physical_grasp/release",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){
   if(!ecm){r->message="NOT_READY";return;}
   if(joint!=sim::kNullEntity)ecm->RequestRemoveEntity(joint);
   joint=sim::kNullEntity;child=sim::kNullEntity;holding.clear();r->success=true;r->message="RELEASED";
   RCLCPP_INFO(node->get_logger(),"PHYSICAL RELEASE");
  });
  status_service=node->create_service<Trigger>("/physical_grasp/status",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){
   r->success=ecm&&joint!=sim::kNullEntity&&ecm->Component<sim::components::DetachableJoint>(joint);
   r->message=holding;
   if(r->success){auto p=(sim::worldPose(palm,*ecm).Inverse()*sim::worldPose(child,*ecm)).Pos();r->success=std::abs(p.X())<.015&&std::abs(p.Y())<.02&&std::abs(p.Z()-.06)<.02;}
  });
 }
 void PreUpdate(const sim::UpdateInfo &,sim::EntityComponentManager &manager) override {
  // ROS retains weak controller callbacks until its executor is destroyed.
  if(!controllers_pinned){
   controllers_pinned=true;
   for(const auto *name:{"libjoint_state_broadcaster.so","libjoint_trajectory_controller.so"}){
    if(void *library=dlopen(name,RTLD_NOW|RTLD_NOLOAD|RTLD_NODELETE))dlclose(library);
    else controllers_pinned=false;
   }
  }
  ecm=&manager;executor.spin_some();
  ecm=nullptr;
 }
};
IGNITION_ADD_PLUGIN(PhysicalGrasp,sim::System,PhysicalGrasp::ISystemConfigure,PhysicalGrasp::ISystemPreUpdate)
