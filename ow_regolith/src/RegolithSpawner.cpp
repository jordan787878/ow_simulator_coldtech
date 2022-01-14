// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// TODO:
//  1. Support sloped scooping.
//  2. Work around reliance on action callbacks from ow_lander.

#include "RegolithSpawner.h"

#include "sdf_utility.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>
#include <gazebo_msgs/LinkStates.h>

using namespace ow_dynamic_terrain;
using namespace ow_lander;
using namespace ow_regolith;
using namespace gazebo_msgs;
using namespace sensor_msgs;
using namespace cv_bridge;
using namespace sdf_utility;
using namespace std::chrono_literals;

using tf::Vector3;
using tf::tfDot;
using tf::quatRotate;
using tf::vector3MsgToTF;
using tf::vector3TFToMsg;
using tf::quaternionMsgToTF;

using std::string;
using std::stringstream;
using std::acos;
using std::cos;
using std::unique_ptr;
using std::find;
using std::distance;
using std::begin;
using std::end;

// service paths used in class
const static string SRV_GET_PHYS_PROPS  = "/gazebo/get_physics_properties";
const static string SRV_SPAWN_MODEL     = "/gazebo/spawn_sdf_model";
const static string SRV_DELETE_MODEL    = "/gazebo/delete_model";
const static string SRV_APPLY_WRENCH    = "/gazebo/apply_body_wrench";
const static string SRV_CLEAR_WRENCH    = "/gazebo/clear_body_wrenches";

// service paths served by this class
const static string SRV_SPAWN_REGOLITH_IN_SCOOP = "/ow_regolith/spawn_regolith_in_scoop";
const static string SRV_REMOVE_ALL_REGOLITH     = "/ow_regolith/remove_all_regolith";

// topic paths used in class
const static string TOPIC_LINK_STATES           = "/gazebo/link_states";
const static string TOPIC_MODIFY_TERRAIN_VISUAL = "/ow_dynamic_terrain/modification_differential/visual";
const static string TOPIC_DIG_LINEAR_RESULT     = "/DigLinear/result";
const static string TOPIC_DIG_CIRCULAR_RESULT   = "/DigCircular/result";
const static string TOPIC_DELIVER_RESULT        = "/Deliver/result";
const static string TOPIC_DISCARD_RESULT        = "/Discard/result";

// constants specific to the scoop end-effector
const static string scoop_link_name  = "lander::l_scoop_tip";
const static Vector3 scoop_forward        = Vector3(1.0, 0.0, 0.0);
const static Vector3 scoop_downward       = Vector3(0.0, 0.0, 1.0);
const static Vector3 scoop_spawn_offset   = Vector3(0.0, 0.0, -0.05);

const static Vector3 world_downward = Vector3(0.0, 0.0, -1.0);

template <class T>
static bool createRosServiceClient(unique_ptr<ros::NodeHandle> &nh,
                                   string service_path, 
                                   ros::ServiceClient &out_srv, 
                                   bool persistent)
{
  // maximum time we will wait for the service to exist
  constexpr auto SERVICE_TIMEOUT = 5.0; // seconds
  out_srv = nh->serviceClient<T>(service_path, persistent);
  if (!out_srv.waitForExistence(ros::Duration(SERVICE_TIMEOUT))) {
    ROS_ERROR("Timed out waiting for service %s to advertise", 
              service_path.c_str());
    return false;
  } 
  return true;
}

template <class T>
static bool callRosService(ros::ServiceClient &srv, T &msg)
{
  if (!srv.isValid()) {
    ROS_ERROR("Connection to service %s has been lost",
              srv.getService().c_str());
    return false;
  }
  if (!srv.call(msg)) {
    ROS_ERROR("Failed to call service %s", srv.getService().c_str());
    return false;
  } 
  return true;
}

RegolithSpawner::RegolithSpawner(ros::NodeHandle* nh)
  : m_node_handle(nh), m_volume_displaced(0.0)
{
  // get node parameters
  if (!m_node_handle->getParam("spawn_volume_threshold", m_spawn_threshold))
    ROS_ERROR("Regolith node requires the spawn_volume_threshold parameter.");
  if (!m_node_handle->getParam("regolith_model_uri", m_model_uri))
    ROS_ERROR("Regolith node requires the regolith_model_uri paramter.");
}

bool RegolithSpawner::initialize()
{
  // set the maximum scoop inclination that the psuedo force can counteract
  // NOTE: do not use a value that makes cosine zero!
  constexpr auto MAX_SCOOP_INCLINATION_DEG = 70.0f; // degrees
  constexpr auto MAX_SCOOP_INCLINATION_RAD = MAX_SCOOP_INCLINATION_DEG * M_PI / 180.0f; // radians
  constexpr auto PSUEDO_FORCE_WEIGHT_FACTOR = 1.0f / cos(MAX_SCOOP_INCLINATION_RAD);

  // load SDF model
  if (!getSdfFromUri(m_model_uri, m_model_sdf)) {
    ROS_ERROR("Failed to load SDF for regolith model");
    return false;
  }
  // parse as SDF object to query properties about the model
  auto sdf = parseSdf(m_model_sdf);
  float model_mass;
  if (!getModelLinkMass(sdf, model_mass) ||
      !getModelLinkName(sdf, m_model_link_name)) {
    ROS_ERROR("Failed to acquire regolith model SDF parameters");
    return false;
  }

  // connect to all ROS services
  ros::ServiceClient gz_get_phys_prop;
  if (!createRosServiceClient<GetPhysicsProperties>(m_node_handle,
                                                    SRV_GET_PHYS_PROPS,
                                                    gz_get_phys_prop,
                                                    false)  ||
      !createRosServiceClient<SpawnModel>(          m_node_handle,
                                                    SRV_SPAWN_MODEL,
                                                    m_gz_spawn_model,
                                                    false)   ||
      !createRosServiceClient<DeleteModel>(         m_node_handle,
                                                    SRV_DELETE_MODEL,
                                                    m_gz_delete_model,
                                                    false)   ||
      !createRosServiceClient<ApplyBodyWrench>(     m_node_handle,
                                                    SRV_APPLY_WRENCH,
                                                    m_gz_apply_wrench,
                                                    false)   ||
      !createRosServiceClient<BodyRequest>(         m_node_handle,
                                                    SRV_CLEAR_WRENCH,
                                                    m_gz_clear_wrench,
                                                    false))
  {
    ROS_ERROR("Failed to connect to all required ROS services");
    return false;
  }

  // advertise services served by this class
  m_spawn_regolith_in_scoop = m_node_handle->advertiseService(
    SRV_SPAWN_REGOLITH_IN_SCOOP, &RegolithSpawner::spawnRegolithInScoopSrv, this);
  m_remove_all_regolith = m_node_handle->advertiseService(
    SRV_REMOVE_ALL_REGOLITH, &RegolithSpawner::removeAllRegolithSrv, this);

  // subscribe to all ROS topics
  m_link_states         = m_node_handle->subscribe(
    TOPIC_LINK_STATES, 1, &RegolithSpawner::onLinkStatesMsg, this);
  m_mod_diff_visual     = m_node_handle->subscribe(
    TOPIC_MODIFY_TERRAIN_VISUAL, 1, &RegolithSpawner::onModDiffVisualMsg, this);
  m_dig_linear_result   = m_node_handle->subscribe(
    TOPIC_DIG_LINEAR_RESULT, 1, &RegolithSpawner::onDigLinearResultMsg, this);
  m_dig_circular_result = m_node_handle->subscribe(
    TOPIC_DIG_CIRCULAR_RESULT, 1, &RegolithSpawner::onDigCircularResultMsg, this);
  m_deliver_result      = m_node_handle->subscribe(
    TOPIC_DELIVER_RESULT, 1, &RegolithSpawner::onDeliverResultMsg, this);
  m_discard_result      = m_node_handle->subscribe(
    TOPIC_DISCARD_RESULT, 1, &RegolithSpawner::onDiscardResultMsg, this);

  // query gazebo for the gravity vector
  GetPhysicsProperties msg;
  if (!callRosService(gz_get_phys_prop, msg)) {
    ROS_ERROR("Failed to query Gazebo for gravity vector");
    return false;
  }
  Vector3 gravity;
  vector3MsgToTF(msg.response.gravity, gravity);
  // psuedo force magnitude = model's weight X weight factor
  m_psuedo_force_mag = model_mass * gravity.length() * PSUEDO_FORCE_WEIGHT_FACTOR;

  return true;
}

bool RegolithSpawner::spawnRegolithInScoop(bool with_pushback)
{
  ROS_INFO("Spawning regolith");

  static auto spawn_count = 0;
  stringstream model_name, body_name;
  model_name << "regolith_" << spawn_count++;
  body_name << model_name.str() << "::" << m_model_link_name;

  SpawnModel spawn_msg;
  
  spawn_msg.request.model_name                  = model_name.str();
  spawn_msg.request.model_xml                   = m_model_sdf;
  spawn_msg.request.robot_namespace             = "/regolith";
  spawn_msg.request.reference_frame             = scoop_link_name;

  spawn_msg.request.initial_pose.orientation.x  = 0.0;
  spawn_msg.request.initial_pose.orientation.y  = 0.0;
  spawn_msg.request.initial_pose.orientation.z  = 0.0;
  spawn_msg.request.initial_pose.orientation.w  = 0.0;

  spawn_msg.request.initial_pose.position.x     = scoop_spawn_offset.getX();
  spawn_msg.request.initial_pose.position.y     = scoop_spawn_offset.getY();
  spawn_msg.request.initial_pose.position.z     = scoop_spawn_offset.getZ();

  if (!callRosService(m_gz_spawn_model, spawn_msg))
    return false;

  m_active_models.push_back({model_name.str(), body_name.str()});

  // if no pushback is desired, we're done
  if (!with_pushback)
    return true;

  // compute scooping direction from scoop orientation
  Vector3 scooping_vec(quatRotate(m_scoop_orientation, scoop_forward));
  // flatten scooping_vec against the X-Y plane
  scooping_vec.setZ(0.0);
  // define pushback direction as opposite to the scooping direction
  Vector3 pushback_vec(-scooping_vec.normalize());

  // apply psuedo force to keep model in the scoop
  ApplyBodyWrench wrench_msg;
  vector3TFToMsg(m_psuedo_force_mag * pushback_vec,
                 wrench_msg.request.wrench.force);

  wrench_msg.request.body_name         = body_name.str();
  
  // Choose a long ros duration (1 year), to delay the disabling of wrench force
  auto one_year_in_seconds = std::chrono::duration_cast<std::chrono::seconds>(1h*24*365).count();
  wrench_msg.request.duration          = ros::Duration(one_year_in_seconds);

  wrench_msg.request.reference_point.x = 0.0;
  wrench_msg.request.reference_point.y = 0.0;
  wrench_msg.request.reference_point.z = 0.0;

  wrench_msg.request.wrench.torque.x   = 0.0;
  wrench_msg.request.wrench.torque.y   = 0.0;
  wrench_msg.request.wrench.torque.z   = 0.0;

  return callRosService(m_gz_apply_wrench, wrench_msg);
}

bool RegolithSpawner::clearAllPsuedoForces()
{
  bool service_call_error_occurred = false;
  BodyRequest msg;
  for (auto &regolith : m_active_models) {
    msg.request.body_name = regolith.body_name;
    if (!callRosService(m_gz_clear_wrench, msg)) {
      ROS_WARN("Failed to clear force on %s", regolith.body_name.c_str());
      service_call_error_occurred = true;
    }
  }
  return !service_call_error_occurred;
}

bool RegolithSpawner::removeAllRegolithModels()
{
  bool service_call_error_occurred = false;
  DeleteModel msg;
  auto it = m_active_models.begin();
  while (it != m_active_models.end()) {
    msg.request.model_name = it->model_name;
    if (callRosService(m_gz_delete_model, msg)) {
      it = m_active_models.erase(it);
    } else {
      ROS_WARN("Failed to delete model %s", it->model_name.c_str());
      ++it;
      service_call_error_occurred = true;
    }
  }
  return !service_call_error_occurred;
}

bool RegolithSpawner::spawnRegolithInScoopSrv(SpawnRegolithInScoopRequest &request,
                                              SpawnRegolithInScoopResponse &response)
{
  response.success = spawnRegolithInScoop(false);
  return response.success;
}

bool RegolithSpawner::removeAllRegolithSrv(RemoveAllRegolithRequest &request,
                                           RemoveAllRegolithResponse &response)
{
  response.success = removeAllRegolithModels();
  return response.success;
}

void RegolithSpawner::onLinkStatesMsg(const LinkStates::ConstPtr &msg) 
{
  auto name_it = find(begin(msg->name), end(msg->name), scoop_link_name);
  if (name_it == end(msg->name)) {
    ROS_WARN("Failed to find %s in link states", scoop_link_name.c_str());      
    return;
  }

  auto pose_it = begin(msg->pose) + distance(begin(msg->name), name_it);
  quaternionMsgToTF(pose_it->orientation, m_scoop_orientation);
}

void RegolithSpawner::onModDiffVisualMsg(const modified_terrain_diff::ConstPtr& msg)
{
  // check if visual terrain modification was caused by the scoop
  Vector3 scoop_bottom(quatRotate(m_scoop_orientation, scoop_downward));
  if (tfDot(scoop_bottom, world_downward) < 0.0)
    // Scoop bottom is pointing up, which is not a proper scooping orientation.
    // This can be the result of a deep grind, and should not result in 
    // particles being spawned.
    return;

  // import image to so we can traverse it
  auto image_handle = CvImageConstPtr();
  try {
    image_handle = toCvShare(msg->diff, msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  auto rows = image_handle->image.rows;
  auto cols = image_handle->image.cols;
  if (rows <= 0 || cols <= 0) {
    ROS_DEBUG("Differential image dimensions are zero or negative");
    return;
  }

  auto pixel_area = (msg->height / rows) * (msg->width / cols);

  // estimate the total volume displaced using a Riemann sum over the image
  for (auto y = 0; y < rows; ++y)
    for (auto x = 0; x < cols; ++x)
      m_volume_displaced += -image_handle->image.at<float>(y, x) * pixel_area;

  if (m_volume_displaced >= m_spawn_threshold) {
    // deduct threshold from tracked volume
    m_volume_displaced -= m_spawn_threshold;
    // spawn a regolith model
    if (!spawnRegolithInScoop(true))
      ROS_ERROR("Failed to spawn regolith in scoop");
  }
}

void RegolithSpawner::onDigLinearResultMsg(const DigLinearActionResult::ConstPtr &msg)
{
  clearAllPsuedoForces();
  // reset to avoid carrying over volume from one digging event to the next
  m_volume_displaced = 0.0;
}

void RegolithSpawner::onDigCircularResultMsg(const DigCircularActionResult::ConstPtr &msg)
{
  clearAllPsuedoForces();
  m_volume_displaced = 0.0;
}

void RegolithSpawner::onDeliverResultMsg(const DeliverActionResult::ConstPtr &msg)
{
  removeAllRegolithModels();
}

void RegolithSpawner::onDiscardResultMsg(const DiscardActionResult::ConstPtr &msg)
{
  removeAllRegolithModels();
}
