#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <gazebo_ros/node.hpp>

#include <memory>
#include <vector>
#include <deque>
#include <mutex>
#include <future>
#include <algorithm>
#include <tuple>
#include <iostream>
#include <unordered_set>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

#include "ros2_linkattacher/gazebo_link_attacher.hpp"   // your header with JointSTRUCT
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/int32.hpp>

// ======================================================================================
// Global list of active attachments (protected by a mutex)
std::vector<JointSTRUCT> GV_joints;
static std::mutex GV_mtx;

namespace gazebo_ros
{

// ======================================================================================
// Command: services enqueue; update thread executes
struct Cmd {
  enum Type { ATTACH, DETACH } type;
  // inputs
  std::string m1, l1, m2, l2, joint_name;
  // sync
  std::shared_ptr<std::promise<void>> done;
};

class GazeboLinkAttacherPrivate
{
public:
  // ROS service handlers (enqueue; optionally wait bounded time)
  void Attach(linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
              linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res);

  void Detach(linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
              linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res);

  // Update-thread queue processor
  void ProcessQueue();

  // State publisher
  void PublishAttachmentState();

  // Lookup (order-agnostic)
  bool getJointEither(const std::string& M1, const std::string& L1,
                      const std::string& M2, const std::string& L2,
                      JointSTRUCT &joint);

  // == Actual work (runs on update thread) ==
  void doAttach(Cmd& cmd);
  void doDetach(Cmd& cmd);

  // Helper functions 
  struct ModelLinkPair {
    gazebo::physics::ModelPtr model;
    gazebo::physics::LinkPtr link;
    bool isRobot;
  };
  
  std::pair<ModelLinkPair, ModelLinkPair> getModelLinkPair(const std::string& m1_name, const std::string& l1_name,
                                                          const std::string& m2_name, const std::string& l2_name);
  
  std::vector<gazebo::physics::LinkPtr> getNonRobotLinks(const ModelLinkPair& pair1, const ModelLinkPair& pair2);
  
  bool isJointAlreadyAttached(const gazebo::physics::LinkPtr& l1, const gazebo::physics::LinkPtr& l2);
  
  // Find joint by model/link names (order-agnostic)
  std::vector<JointSTRUCT>::iterator findJoint(const std::string& m1, const std::string& l1,
                                              const std::string& m2, const std::string& l2);

  // Heuristic: identify UR robot models so we never freeze/damp them
  static bool isRobotModel(const gazebo::physics::ModelPtr& m) {
    if (!m) return false;
    const std::string n = m->GetName();
    // Check if name starts with "ur" 
    return n.rfind("ur", 0) == 0;
  }

  // ===== Freeze / Unfreeze ============================================================
  // Freeze non-robot links to keep them perfectly still.
  // Freeze = zero vels, gravity off, kinematic true. Unfreeze restores.
  void freezeLink(const gazebo::physics::ModelPtr& m, const gazebo::physics::LinkPtr& L);
  void unfreezeLink(const gazebo::physics::ModelPtr& m, const gazebo::physics::LinkPtr& L);
  bool isFrozen(const std::string& model, const std::string& link) const;

  std::unordered_set<std::string> frozen_;  // "model_name::link_name"
  mutable std::mutex frozen_mtx_;

  // Scoped, light stabilization during ops (temporary damping only)
  struct ScopedStabilize {
    std::vector<std::pair<gazebo::physics::LinkPtr,
      std::tuple<bool,double,double>>> saved;
    explicit ScopedStabilize(const std::vector<gazebo::physics::LinkPtr>& links){
      saved.reserve(links.size());
      for (auto &L : links) if (L) {
        saved.push_back({L, {L->GetGravityMode(), L->GetLinearDamping(), L->GetAngularDamping()}});
        L->SetLinearVel(ignition::math::Vector3d::Zero);
        L->SetAngularVel(ignition::math::Vector3d::Zero);
        // temporarily calm contacts; we do NOT change gravity/kinematic here
        L->SetLinearDamping(3.0);
        L->SetAngularDamping(3.0);
      }
    }
    ~ScopedStabilize(){
      for (auto &s : saved) {
        auto L=s.first; auto [g,ld,ad]=s.second;
        if (!L) continue;
        L->SetLinearDamping(ld);
        L->SetAngularDamping(ad);
      }
    }
  };

  // World + ROS
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;

  // Services & publisher
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_service_;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_service_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr attachment_state_publisher_;
  rclcpp::TimerBase::SharedPtr attachment_state_timer_;

  // Update-thread queue
  std::mutex q_mtx_;
  std::deque<Cmd> queue_;
  gazebo::event::ConnectionPtr update_conn_;
};

// ======================================================================================
// Plugin shell

GazeboLinkAttacher::GazeboLinkAttacher()
: impl_(std::make_unique<GazeboLinkAttacherPrivate>()) {}

GazeboLinkAttacher::~GazeboLinkAttacher() = default;

void GazeboLinkAttacher::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  impl_->world_ = _world;
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Services
  impl_->attach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::AttachLink>(
      "ATTACHLINK",
      std::bind(&GazeboLinkAttacherPrivate::Attach, impl_.get(),
                std::placeholders::_1, std::placeholders::_2));

  impl_->detach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::DetachLink>(
      "DETACHLINK",
      std::bind(&GazeboLinkAttacherPrivate::Detach, impl_.get(),
                std::placeholders::_1, std::placeholders::_2));

  // Publisher: count of active joints
  impl_->attachment_state_publisher_ =
    impl_->ros_node_->create_publisher<std_msgs::msg::Int32>("attachment_state", 10);

  // Timer (100 ms)
  impl_->attachment_state_timer_ =
    impl_->ros_node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GazeboLinkAttacherPrivate::PublishAttachmentState, impl_.get()));

  // Process queued commands on the Gazebo update thread
  impl_->update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboLinkAttacherPrivate::ProcessQueue, impl_.get()));
}

// ======================================================================================
// Helpers

bool GazeboLinkAttacherPrivate::getJointEither(const std::string& M1, const std::string& L1,
                                               const std::string& M2, const std::string& L2,
                                               JointSTRUCT &joint)
{
  std::lock_guard<std::mutex> lk(GV_mtx);
  auto it = findJoint(M1, L1, M2, L2);
  if (it != GV_joints.end()) {
    joint = *it;
    return true;
  }
  return false;
}

void GazeboLinkAttacherPrivate::PublishAttachmentState()
{
  std_msgs::msg::Int32 msg;
  {
    std::lock_guard<std::mutex> lk(GV_mtx);
    msg.data = static_cast<int32_t>(GV_joints.size());
  }
  attachment_state_publisher_->publish(msg);
}

// Freeze/unfreeze implementation
void GazeboLinkAttacherPrivate::freezeLink(const gazebo::physics::ModelPtr& m,
                                           const gazebo::physics::LinkPtr& L)
{
  if (!m || !L) return;
  if (isRobotModel(m)) return; // never freeze robot links

  std::string key = m->GetName() + "::" + L->GetName();
  {
    std::lock_guard<std::mutex> lk(frozen_mtx_);
    frozen_.insert(key);
  }

  // Apply freeze
  L->SetLinearVel(ignition::math::Vector3d::Zero);
  L->SetAngularVel(ignition::math::Vector3d::Zero);
  L->SetGravityMode(false);
  // Kinematic: non-dynamic but keeps collisions; stays exactly where it is
  L->SetKinematic(true);
}

void GazeboLinkAttacherPrivate::unfreezeLink(const gazebo::physics::ModelPtr& m,
                                             const gazebo::physics::LinkPtr& L)
{
  if (!m || !L) return;
  if (isRobotModel(m)) return;

  std::string key = m->GetName() + "::" + L->GetName();
  {
    std::lock_guard<std::mutex> lk(frozen_mtx_);
    if (frozen_.find(key) == frozen_.end()) return; // not frozen
    frozen_.erase(key);
  }

  // Restore normal dynamics
  L->SetKinematic(false);
  L->SetGravityMode(true);
}

bool GazeboLinkAttacherPrivate::isFrozen(const std::string& model, const std::string& link) const
{
  std::string key = model + "::" + link;
  std::lock_guard<std::mutex> lk(frozen_mtx_);
  return frozen_.count(key) > 0;
}

// Helper function implementations
std::pair<GazeboLinkAttacherPrivate::ModelLinkPair, GazeboLinkAttacherPrivate::ModelLinkPair> 
GazeboLinkAttacherPrivate::getModelLinkPair(const std::string& m1_name, const std::string& l1_name,
                                           const std::string& m2_name, const std::string& l2_name)
{
  auto m1 = world_->ModelByName(m1_name);
  auto m2 = world_->ModelByName(m2_name);
  if (!m1 || !m2) {
    throw std::runtime_error("Model not found");
  }

  auto l1 = m1->GetLink(l1_name);
  auto l2 = m2->GetLink(l2_name);
  if (!l1 || !l2) {
    throw std::runtime_error("Link not found");
  }

  ModelLinkPair pair1{m1, l1, isRobotModel(m1)};
  ModelLinkPair pair2{m2, l2, isRobotModel(m2)};
  
  return {pair1, pair2};
}

std::vector<gazebo::physics::LinkPtr> 
GazeboLinkAttacherPrivate::getNonRobotLinks(const ModelLinkPair& pair1, const ModelLinkPair& pair2)
{
  std::vector<gazebo::physics::LinkPtr> links;
  if (!pair1.isRobot) links.push_back(pair1.link);
  if (!pair2.isRobot) links.push_back(pair2.link);
  return links;
}

bool GazeboLinkAttacherPrivate::isJointAlreadyAttached(const gazebo::physics::LinkPtr& l1, const gazebo::physics::LinkPtr& l2)
{
  std::lock_guard<std::mutex> lk(GV_mtx);
  for (auto &j: GV_joints) {
    if ((j.l1==l1 && j.l2==l2) || (j.l1==l2 && j.l2==l1)) {
      return true;
    }
  }
  return false;
}

std::vector<JointSTRUCT>::iterator 
GazeboLinkAttacherPrivate::findJoint(const std::string& m1, const std::string& l1,
                                    const std::string& m2, const std::string& l2)
{
  return std::find_if(GV_joints.begin(), GV_joints.end(), [&](const JointSTRUCT& e){
    bool f = (e.model1==m1 && e.link1==l1 && e.model2==m2 && e.link2==l2);
    bool r = (e.model1==m2 && e.link1==l2 && e.model2==m1 && e.link2==l1);
    return f || r;
  });
}

// ======================================================================================
// Service callbacks (enqueue work, then wait bounded time)

void GazeboLinkAttacherPrivate::Attach(
  linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res)
{
  auto p = std::make_shared<std::promise<void>>();
  Cmd cmd;
  cmd.type = Cmd::ATTACH;
  cmd.m1 = _req->model1_name; cmd.l1 = _req->link1_name;
  cmd.m2 = _req->model2_name; cmd.l2 = _req->link2_name;
  cmd.joint_name = cmd.m1 + "_" + cmd.l1 + "_" + cmd.m2 + "_" + cmd.l2 + "_joint";
  cmd.done = p;

  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    queue_.push_back(std::move(cmd));
  }

  auto fut = p->get_future();
  if (fut.wait_for(std::chrono::milliseconds(1500)) != std::future_status::ready) {
    _res->success = false;
    _res->message = "Attach queued; processing timeout (will complete asynchronously).";
    return;
  }

  // Verify: pair should now exist
  JointSTRUCT j;
  bool ok = getJointEither(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j);
  _res->success = ok;
  _res->message = ok ?
    ("ATTACHED: {" + _req->model1_name + " , " + _req->link1_name +
     "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.")
    : "Attach failed (see Gazebo log).";
}

void GazeboLinkAttacherPrivate::Detach(
  linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res)
{
  auto p = std::make_shared<std::promise<void>>();
  Cmd cmd;
  cmd.type = Cmd::DETACH;
  cmd.m1 = _req->model1_name; cmd.l1 = _req->link1_name;
  cmd.m2 = _req->model2_name; cmd.l2 = _req->link2_name;
  cmd.done = p;

  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    queue_.push_back(std::move(cmd));
  }

  auto fut = p->get_future();
  if (fut.wait_for(std::chrono::milliseconds(1500)) != std::future_status::ready) {
    _res->success = false;
    _res->message = "Detach queued; processing timeout (will complete asynchronously).";
    return;
  }

  // Idempotent: if it doesn't exist anymore, we're done
  JointSTRUCT j;
  bool stillThere = getJointEither(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j);
  _res->success = !stillThere;
  _res->message = stillThere ? "Detach failed (see Gazebo log)." : "DETACHED (or already detached).";
}

// ======================================================================================
// Update-thread queue processor

void GazeboLinkAttacherPrivate::ProcessQueue()
{
  for (;;) {
    Cmd cmd;
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      if (queue_.empty()) break;
      cmd = std::move(queue_.front());
      queue_.pop_front();
    }

    if (cmd.type == Cmd::ATTACH)      doAttach(cmd);
    else /* DETACH */                  doDetach(cmd);

    if (cmd.done) cmd.done->set_value();
  }
}

// ======================================================================================
// Actual attach/detach (runs on update thread ONLY)

void GazeboLinkAttacherPrivate::doAttach(Cmd& cmd)
{
  try {
    auto [pair1, pair2] = getModelLinkPair(cmd.m1, cmd.l1, cmd.m2, cmd.l2);
    
    // Unfreeze links if they were frozen earlier
    if (isFrozen(cmd.m1, cmd.l1)) unfreezeLink(pair1.model, pair1.link);
    if (isFrozen(cmd.m2, cmd.l2)) unfreezeLink(pair2.model, pair2.link);

    // Prevent duplicate pair only (multi-attach allowed across different pairs)
    if (isJointAlreadyAttached(pair1.link, pair2.link)) {
      std::cerr << "[LinkAttacher] Pair already attached\n";
      return;
    }

    // Light stabilization during op (temporary damping only)
    auto to_stabilize = getNonRobotLinks(pair1, pair2);
    ScopedStabilize S(to_stabilize);

    auto joint = pair1.model->CreateJoint(cmd.joint_name, "fixed", pair1.link, pair2.link);
    if (!joint) { std::cerr << "[LinkAttacher] CreateJoint failed\n"; return; }

    joint->Attach(pair1.link, pair2.link);
    joint->Load(pair1.link, pair2.link, ignition::math::Pose3d());  // identity pose
    joint->SetProvideFeedback(true);
    joint->Init();

    JointSTRUCT rec;
    rec.model1 = cmd.m1; rec.m1 = pair1.model; rec.link1 = cmd.l1; rec.l1 = pair1.link;
    rec.model2 = cmd.m2; rec.m2 = pair2.model; rec.link2 = cmd.l2; rec.l2 = pair2.link;
    rec.joint  = joint;

    {
      std::lock_guard<std::mutex> lk(GV_mtx);
      GV_joints.push_back(rec);
    }
    PublishAttachmentState();

    // For object↔object assemblies, keep them parked (frozen) after attach
    if (!pair1.isRobot && !pair2.isRobot) {
      freezeLink(pair1.model, pair1.link);
      freezeLink(pair2.model, pair2.link);
    }
  } catch (const std::runtime_error& e) {
    std::cerr << "[LinkAttacher] " << e.what() << "\n";
  }
}

void GazeboLinkAttacherPrivate::doDetach(Cmd& cmd)
{
  JointSTRUCT j;
  {
    std::lock_guard<std::mutex> lk(GV_mtx);
    auto it = findJoint(cmd.m1, cmd.l1, cmd.m2, cmd.l2);
    if (it == GV_joints.end()) {
      // Already detached — ok
      return;
    }
    j = *it;
  }

  // Light stabilization during joint removal
  std::vector<gazebo::physics::LinkPtr> to_stabilize;
  if (!isRobotModel(j.m1)) to_stabilize.push_back(j.l1);
  if (!isRobotModel(j.m2)) to_stabilize.push_back(j.l2);
  ScopedStabilize S(to_stabilize);

  if (j.joint) j.joint->Detach();
  // Try remove from both models (joint was created on m1, but be safe)
  if (j.m1) j.m1->RemoveJoint(j.joint ? j.joint->GetName() : "");
  if (j.m2) j.m2->RemoveJoint(j.joint ? j.joint->GetName() : "");

  {
    std::lock_guard<std::mutex> lk(GV_mtx);
    GV_joints.erase(std::remove_if(GV_joints.begin(), GV_joints.end(),
                    [&](const JointSTRUCT& e){ return e.joint == j.joint; }),
                    GV_joints.end());
  }
  PublishAttachmentState();

  // Persistently freeze the now-detached non-robot links so they don't move at all
  if (!isRobotModel(j.m1)) freezeLink(j.m1, j.l1);
  if (!isRobotModel(j.m2)) freezeLink(j.m2, j.l2);
}

// ======================================================================================
// Register plugin

GZ_REGISTER_WORLD_PLUGIN(GazeboLinkAttacher)

} // namespace gazebo_ros
