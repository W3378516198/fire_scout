#include "fire_scout/passage_scheduler.hpp"
#include "fire_scout/ros_utils.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>

using namespace fire_scout;
// Low-bandwidth capacity-one admission service for automatically detected
// bottlenecks. Requests/leases are heartbeats, not static obstacle map edits.
class PassageCoordinator : public rclcpp::Node {
public:
  PassageCoordinator():Node("passage_coordinator"),tf_(get_clock()),listener_(tf_){
    world_=declare_parameter<std::string>("world_frame","world");
    const auto names=declare_parameter<std::vector<std::string>>("scout_names",
      std::vector<std::string>{"scout1","scout2","scout3","scout4"});
    scheduler_.merge_radius=declare_parameter("passage_merge_radius",1.4);
    scheduler_.request_timeout=declare_parameter("passage_request_timeout",.8);
    scheduler_.exit_radius=declare_parameter("passage_exit_radius",1.8);
    if(names.empty()||names.size()>32||world_.empty()||
       !std::isfinite(scheduler_.merge_radius)||scheduler_.merge_radius<.2||
       scheduler_.merge_radius>3||!std::isfinite(scheduler_.request_timeout)||
       scheduler_.request_timeout<.2||scheduler_.request_timeout>3||
       !std::isfinite(scheduler_.exit_radius)||scheduler_.exit_radius<=scheduler_.merge_radius||
       scheduler_.exit_radius>6)throw std::runtime_error("Invalid passage coordinator parameters");
    for(const auto &n:names){
      if(n.empty()||std::count(names.begin(),names.end(),n)!=1)
        throw std::runtime_error("scout_names must be nonempty and unique");
    }
    requests_.resize(names.size());
    for(size_t i=0;i<names.size();++i){
      grants_.push_back(create_publisher<nav_msgs::msg::Path>(
        "/"+names[i]+"/planning/passage_grant",rclcpp::QoS(1).reliable()));
      subscriptions_.push_back(create_subscription<nav_msgs::msg::Path>(
        "/"+names[i]+"/planning/passage_request",rclcpp::QoS(1).reliable(),
        [this,i](nav_msgs::msg::Path::SharedPtr m){receive(i,*m);}));
    }
    status_=create_publisher<std_msgs::msg::String>("/fire_scout/coordination/status",
      rclcpp::QoS(1).reliable());
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{tick();});
    config_lock_=lockParameters(*this);
    RCLCPP_INFO(get_logger(),"V2.1.8 passage coordinator: scouts=%zu FIFO leases, measured-owner migration",names.size());
  }
private:
  void receive(size_t i,const nav_msgs::msg::Path&m){
    if(m.header.frame_id.empty()||(m.poses.size()!=1&&m.poses.size()!=3))return;
    const double now_s=now().seconds();const double source=double(stampNs(m.header.stamp))*1e-9;
    if(source<=0||source>now_s+.05||now_s-source>scheduler_.request_timeout)return;
    if(stampNs(m.header.stamp)<stampNs(requests_[i].header.stamp))return;
    tf2::Transform transform;transform.setIdentity();
    try{if(m.header.frame_id!=world_){
      auto t=tf_.lookupTransform(world_,m.header.frame_id,rclcpp::Time(0,0,get_clock()->get_clock_type()));
      const auto&q=t.transform.rotation;
      const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
      if(!std::isfinite(n)||n<1e-6)return;
      transform.setRotation(tf2::Quaternion(q.x/n,q.y/n,q.z/n,q.w/n));
      transform.setOrigin(tf2::Vector3(t.transform.translation.x,t.transform.translation.y,t.transform.translation.z));
    }}catch(const tf2::TransformException&){return;}
    std::vector<Vec3>p;
    for(const auto&pose:m.poses){const auto&v=pose.pose.position;
      if(!finite({v.x,v.y,v.z}))return;
      auto w=transform*tf2::Vector3(v.x,v.y,v.z);p.push_back({w.x(),w.y(),w.z()});}
    if(p.size()==1){requests_[i]=m;scheduler_.release(int(i),p[0]);return;}
    const double radius=distance(p[2],p[0]);
    const int64_t epoch=stampNs(m.poses[0].header.stamp);
    if(!std::isfinite(radius)||radius<.2||radius>6||epoch<=0)return;
    requests_[i]=m;
    // The scheduler reconciles the old physical lease on EVERY fresh owner
    // position, including heartbeats after an initially rejected migration.
    scheduler_.request({int(i),p[0],p[1],now_s,now_s,distance(p[0],p[1])<.65,radius,epoch});
  }
  void tick(){
    const double t=now().seconds();
    // Keep ownership across a /clock rewind, quarantine all owners. Dropping
    // leases here would silently declare occupied doors empty after reset.
    if(last_time_>0&&t<last_time_-.05){
      scheduler_.bids.clear();scheduler_.completed.clear();for(auto &l:scheduler_.leases)l.quarantined=true;
      for(auto &r:requests_)r=nav_msgs::msg::Path{};
    }
    last_time_=t;scheduler_.update(t);
    for(size_t i=0;i<grants_.size();++i){
      nav_msgs::msg::Path grant;grant.header=requests_[i].header;
      if(scheduler_.admitted(int(i))&&requests_[i].poses.size()==3)
        grant.poses.push_back(requests_[i].poses[0]);
      else if(requests_[i].poses.size()==3){
        auto done=scheduler_.completed.find(int(i));
        if(done!=scheduler_.completed.end()&&
           done->second.epoch==stampNs(requests_[i].poses[0].header.stamp)){
          // Two poses explicitly acknowledge completion. Echo the request's
          // anchor id and fresh measured position in the UAV's own frame.
          grant.poses.push_back(requests_[i].poses[0]);
          grant.poses.push_back(requests_[i].poses[1]);
        }
      }
      grants_[i]->publish(grant);
    }
    if(t-last_status_>.5){last_status_=t;
      std::ostringstream s;s<<"runtime=2.1.8 requests="<<scheduler_.bids.size()
        <<" resources="<<scheduler_.leases.size()<<" completed="<<scheduler_.completed.size()
        <<" migrations_completed="<<scheduler_.migrations_completed;
      for(size_t i=0;i<scheduler_.leases.size();++i){auto &l=scheduler_.leases[i];
        s<<" portal"<<i<<"="<<l.owner<<":"<<(l.quarantined?"QUARANTINED":"LEASE")
          <<" entry_seen"<<i<<"="<<l.entered<<" exit_radius"<<i<<"="<<l.release_radius
          <<" owner_anchor"<<i<<"="<<l.owner_anchor.x<<","<<l.owner_anchor.y<<","<<l.owner_anchor.z
          <<" epoch"<<i<<"="<<l.epoch;}
      status_->publish(textMessage(s.str()));
    }
  }
  PassageScheduler scheduler_;std::string world_;double last_time_{-1},last_status_{-1};
  std::vector<nav_msgs::msg::Path>requests_;
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr>subscriptions_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>grants_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_lock_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PassageCoordinator>());rclcpp::shutdown();return 0;}
