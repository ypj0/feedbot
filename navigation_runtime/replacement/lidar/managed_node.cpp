#include "sdk_control.hpp"
#include "udp_input.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sys/file.h>
#include <fcntl.h>
#include <fstream>
#include <thread>
#include <cstdio>
#include <atomic>
#include <mutex>
using Callback=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class Driver:public rclcpp_lifecycle::LifecycleNode {
 std::unique_ptr<lidar::SdkControl> control;std::unique_ptr<lidar::Lifecycle> core;std::unique_ptr<lidar::UdpInput> udp;
 lidar::Assembler assembler;int lock=-1;std::string directory,token;bool operational=false;uint64_t scans=0;double last_status=0,last_check=0;
 struct Captured {lidar::Scan scan;double end_stamp;double received;uint64_t sequence;};
 std::atomic<bool> reading{false};std::thread reader;std::mutex capture_mutex;
 std::shared_ptr<const Captured> latest;std::string reader_error;uint64_t consumed=0;
 void stop_reader(){reading=false;if(reader.joinable())reader.join();}
 void start_reader(){
  {std::lock_guard<std::mutex> guard(capture_mutex);latest.reset();reader_error.clear();consumed=0;}
  reading=true;
  reader=std::thread([this]{uint64_t sequence=0;try{
   while(reading){
    lidar::Block b;double stamp;
    if(!udp->read(b,stamp)){std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;}
    lidar::Scan scan;if(!assembler.add(b,stamp,scan))continue;
    double age=get_clock()->now().seconds()-stamp;
    if(age<-.01||age>.2)throw std::runtime_error("UDP source age="+std::to_string(age));
    auto next=std::make_shared<Captured>(Captured{std::move(scan),stamp,mono()-std::max(0.0,age),++sequence});
    {std::lock_guard<std::mutex> guard(capture_mutex);latest=std::move(next);}
   }
  }catch(const std::exception& e){std::lock_guard<std::mutex> guard(capture_mutex);reader_error=e.what();reading=false;}});
 }
 void publish_latest(){
  std::shared_ptr<const Captured> item;std::string error;
  {std::lock_guard<std::mutex> guard(capture_mutex);item=latest;error=reader_error;}
  if(!error.empty())throw std::runtime_error(error);
  if(!item||item->sequence==consumed)return;
  const double now=mono();
  if(now-item->received>.2)throw std::runtime_error("complete scan source stale");
  consumed=item->sequence;
  if(!core->frame(core->epoch(),item->received,now))return;
  const auto & s=item->scan;
  sensor_msgs::msg::LaserScan m;m.header.frame_id="scan";m.header.stamp=rclcpp::Time(int64_t(s.stamp*1e9));m.angle_min=s.angle_min;m.angle_max=s.angle_max;m.angle_increment=(s.angle_max-s.angle_min)/(s.ranges.size()-1);m.time_increment=s.increment;m.scan_time=s.period;m.range_min=.05;m.range_max=40;m.ranges=s.ranges;m.intensities=s.intensities;
  scan_pub->publish(m);scans++;
 }
 rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub;rclcpp::TimerBase::SharedPtr timer;
 bool permitted(){std::ifstream f(directory+"/permit");std::string s;std::getline(f,s);return f.good()&&s==token&&!token.empty();}
 void status(bool force=false){if(!force&&mono()-last_status<.1)return;last_status=mono();std_msgs::msg::Bool b;b.data=operational&&core&&core->ready();try{ready_pub->publish(b);}catch(const std::exception&){}
  if(directory.empty()||lock<0)return;std::ofstream f(directory+"/status.tmp");f<<"{\"ready\":"<<(b.data?"true":"false")<<",\"healthy\":"<<((core&&core->state()!=lidar::State::Fault&&permitted())?"true":"false")<<",\"heartbeat\":"<<mono()<<",\"pid\":"<<getpid()<<",\"scans\":"<<scans<<"}\n";f.close();std::rename((directory+"/status.tmp").c_str(),(directory+"/status.json").c_str());
 }
 void clear(){operational=false;stop_reader();status(true);if(core){try{core->release();}catch(...){}}udp.reset();core.reset();control.reset();if(lock>=0){flock(lock,LOCK_UN);close(lock);lock=-1;}}
 bool receive(bool publish){lidar::Block b;double stamp;if(!udp->read(b,stamp))return false;lidar::Scan s;if(!assembler.add(b,stamp,s))return true;
  double age=get_clock()->now().seconds()-stamp,now=mono();if(age<-.01||age>.2)throw std::runtime_error("stale/source clock step");
  if(!core->frame(core->epoch(),now-std::max(0.0,age),now))return true;
  if(publish){sensor_msgs::msg::LaserScan m;m.header.frame_id="scan";m.header.stamp=rclcpp::Time(int64_t(s.stamp*1e9));m.angle_min=s.angle_min;m.angle_max=s.angle_max;m.angle_increment=(s.angle_max-s.angle_min)/(s.ranges.size()-1);m.time_increment=s.increment;m.scan_time=s.period;m.range_min=.05;m.range_max=40;m.ranges=s.ranges;m.intensities=s.intensities;scan_pub->publish(m);scans++;}return true;
 }
 void poll(){try{if(!core)return;if(!permitted()){operational=false;stop_reader();core->ownership_lost();status(true);return;}if(!operational&&core->state()!=lidar::State::Fault&&mono()-last_check>1){last_check=mono();control->validate();}if(operational){publish_latest();core->tick(mono());if(!core->ready()){operational=false;stop_reader();}}status();}catch(const std::exception& e){operational=false;stop_reader();if(core)core->transport_lost();RCLCPP_ERROR(get_logger(),"%s",e.what());status(true);}}
 public:
 Driver():LifecycleNode("ltr2_driver"){
  declare_parameter("startup_timeout",5.0);declare_parameter("device_address","192.168.10.160");declare_parameter("device_model","LT-R2-B");declare_parameter("target_address","255.255.255.255");declare_parameter("control_port",2105);declare_parameter("data_port",2105);declare_parameter("owner_directory","/run/feedbot-lidar");declare_parameter("owner_token","");
  scan_pub=create_publisher<sensor_msgs::msg::LaserScan>("scan",rclcpp::SensorDataQoS());
  // Ordinary publisher remains available when the lifecycle scan publisher is inactive.
  ready_pub=rclcpp::create_publisher<std_msgs::msg::Bool>(this,"lidar_ready",rclcpp::QoS(1).transient_local().reliable());timer=create_wall_timer(std::chrono::milliseconds(5),[this]{poll();});
 }
 ~Driver(){clear();}
 Callback on_configure(const rclcpp_lifecycle::State&)override{try{
  directory=get_parameter("owner_directory").as_string();token=get_parameter("owner_token").as_string();if(!permitted())throw std::runtime_error("owner permit absent");
  int candidate=open((directory+"/hardware.lock").c_str(),O_CREAT|O_RDWR,0600);if(candidate<0)throw std::runtime_error("hardware lock open");if(flock(candidate,LOCK_EX|LOCK_NB)){close(candidate);throw std::runtime_error("hardware lock busy");}lock=candidate;
  auto ip=get_parameter("device_address").as_string();int dp=get_parameter("data_port").as_int(),cp=get_parameter("control_port").as_int();if(dp<1||dp>65535||cp<1||cp>65535)throw std::runtime_error("port");
  udp.reset(new lidar::UdpInput(ip,dp));control.reset(new lidar::SdkControl(ip,get_parameter("target_address").as_string(),cp,dp,get_parameter("device_model").as_string()));core.reset(new lidar::Lifecycle(*control,mono,get_parameter("startup_timeout").as_double()));core->claim();if(!core->configure())throw std::runtime_error("configure: "+core->last_error());status(true);return Callback::SUCCESS;
 }catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"%s",e.what());clear();return Callback::FAILURE;}}
 Callback on_activate(const rclcpp_lifecycle::State&)override{try{
  if(!core||!permitted())throw std::runtime_error("not configured/owned");assembler=lidar::Assembler();udp->drain();if(!core->activate(mono()))throw std::runtime_error("activate: "+core->last_error());double until=mono()+get_parameter("startup_timeout").as_double();
  while(mono()<until&&!core->ready()){if(!permitted())throw std::runtime_error("permit revoked");receive(false);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  if(!core->ready())throw std::runtime_error("no fresh complete scan");scan_pub->on_activate();start_reader();operational=true;status(true);return Callback::SUCCESS;
 }catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"%s",e.what());operational=false;stop_reader();if(core)core->stop(true);status(true);return Callback::FAILURE;}}
 Callback on_deactivate(const rclcpp_lifecycle::State&)override{operational=false;stop_reader();status(true);scan_pub->on_deactivate();try{return core&&core->stop(true)?Callback::SUCCESS:Callback::ERROR;}catch(...){return Callback::ERROR;}}
 Callback on_cleanup(const rclcpp_lifecycle::State&)override{clear();return Callback::SUCCESS;}
 Callback on_shutdown(const rclcpp_lifecycle::State&)override{clear();return Callback::SUCCESS;}
 Callback on_error(const rclcpp_lifecycle::State&)override{clear();return Callback::SUCCESS;}
};
int main(int argc,char** argv){rclcpp::init(argc,argv);{auto node=std::make_shared<Driver>();rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node->get_node_base_interface());exec.spin();}rclcpp::shutdown();}
