// Workspace backport of MoveIt shutdown fixes: moveit/moveit2 PRs #2001 and #3828.
// Preserve Humble's ABI and destroy plugin users before unloading their libraries.
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group/move_group_context.h>
#include <dlfcn.h>

void moveit_cpp::MoveItCpp::clearContents()
{
  trajectory_execution_manager_.reset();
  planning_pipelines_.clear();
  robot_model_.reset();
  planning_scene_monitor_.reset();
}

trajectory_execution_manager::TrajectoryExecutionManager::~TrajectoryExecutionManager()
{
  run_continuous_execution_thread_ = false;
  stopExecution(true);
  if (private_executor_)
    private_executor_->cancel();
  if (private_executor_thread_.joinable())
    private_executor_thread_.join();
  callback_handler_.reset();
  if (private_executor_ && controller_mgr_node_)
    private_executor_->remove_node(controller_mgr_node_);
  private_executor_.reset();
  controller_manager_.reset();
  controller_mgr_node_.reset();
}

move_group::MoveGroupContext::~MoveGroupContext()
{
  // The main ROS node still owns weak callback references into this capability library.
  // Keep its code mapped until process exit, when the OS reclaims it.
  if (void* library = dlopen("libmoveit_move_group_default_capabilities.so",
                            RTLD_NOW | RTLD_NOLOAD | RTLD_NODELETE))
    dlclose(library);
  plan_execution_.reset();
  trajectory_execution_manager_.reset();
  planning_pipeline_.reset();
  planning_scene_monitor_.reset();
}
