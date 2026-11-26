#pragma once
#include <stdexcept>
#include <cstdint>
#include <string>
#include <cmath>
#include <functional>
namespace lidar {
// All methods are called by one control owner; socket receive only queues frame events.
struct Control {
 virtual ~Control()=default;
 virtual void open()=0;
 virtual void validate()=0;
 virtual void measurement(bool)=0;
 virtual void streaming(bool)=0;
 virtual void close() noexcept=0;
};
enum class State {Disconnected,Inactive,Starting,Active,Sleeping,Fault};
class Lifecycle {
 std::string error;
 Control& device;std::function<double()> clock_;double startup_timeout_;State current=State::Disconnected;bool owned=false;uint64_t generation=0;double deadline=0,last_frame=0;
 void require_owner(){if(!owned)throw std::logic_error("exclusive ownership absent");}
 void fault() noexcept {current=State::Fault;last_frame=0;device.close();}
 public:
 explicit Lifecycle(Control& d,std::function<double()> clock={},double startup_timeout=1.):device(d),clock_(clock),startup_timeout_(startup_timeout){if(!std::isfinite(startup_timeout_)||startup_timeout_<=0||startup_timeout_>30)throw std::invalid_argument("startup timeout");}
 State state()const{return current;}
 const std::string& last_error()const{return error;}
 uint64_t epoch()const{return generation;}
 bool ready()const{return current==State::Active;}
 // Caller must establish real hardware ownership before granting this capability.
 // A local flag/lock does not exclude the unmodified factory driver.
 void claim(){if(current!=State::Disconnected||owned)throw std::logic_error("claim state");owned=true;}
 bool configure(){require_owner();if(current!=State::Disconnected&&current!=State::Fault)throw std::logic_error("configure state");
  generation++;last_frame=0;
  try{device.open();device.validate();current=State::Inactive;error.clear();return true;}catch(const std::exception& e){error=e.what();fault();return false;}catch(...){error="unknown configure error";fault();return false;}
 }
 bool activate(double now){require_owner();if(!std::isfinite(now)||now<0)throw std::logic_error("invalid clock");if(current!=State::Inactive&&current!=State::Sleeping)throw std::logic_error("activate state");
  generation++;last_frame=0;current=State::Starting;deadline=now+startup_timeout_;
  try{device.measurement(true);device.streaming(true);
   // A bounded control RPC can take longer than one scan period on cold start.
   // Fresh-data readiness begins after both control acknowledgements, not before them.
   double acknowledged=clock_?clock_():now;
   if(!std::isfinite(acknowledged)||acknowledged<now)throw std::runtime_error("activation clock discontinuity");
   deadline=acknowledged+startup_timeout_;error.clear();return true;}catch(const std::exception& e){error=e.what();fault();return false;}catch(...){error="unknown activate error";fault();return false;}
 }
 bool frame(uint64_t epoch,double received,double now){
  if(!std::isfinite(received)||!std::isfinite(now)||received<deadline-startup_timeout_||!owned||epoch!=generation||(current!=State::Starting&&current!=State::Active)||received>now||now-received>.2||received<=last_frame)return false;
  if(current==State::Starting&&now>deadline){fault();return false;}
  last_frame=received;current=State::Active;return true;
 }
 void tick(double now){if(!std::isfinite(now)){fault();return;}if((current==State::Starting&&now>deadline)||(current==State::Active&&now-last_frame>.3))fault();}
 bool stop(bool sleep){require_owner();if(current==State::Disconnected)return true;
  // Revoke readiness before any blocking command; attempt both stops independently.
  current=State::Fault;last_frame=0;generation++;bool ok=true;
  try{device.streaming(false);}catch(...){ok=false;}
  try{device.measurement(false);}catch(...){ok=false;}
  if(ok)current=sleep?State::Sleeping:State::Inactive;else device.close();return ok;
 }
 bool release(){require_owner();bool ok=stop(false);device.close();owned=false;current=State::Disconnected;return ok;}
 // Lost lease or transport invalidates immediately; cannot truthfully claim hardware stopped.
 void ownership_lost() noexcept {owned=false;generation++;fault();}
 void transport_lost() noexcept {generation++;fault();}
 // Reconnect configures only. Resuming measurement requires a new explicit activate request.
};
}
