#include "lifecycle.hpp"
#include <vector>
#include <iostream>
#include <functional>
#include <limits>
struct Fake:lidar::Control {
 std::vector<std::string> calls;std::string fail;
 void call(std::string s){calls.push_back(s);if(s==fail)throw std::runtime_error(s);}
 void open()override{call("open");}void validate()override{call("validate");}
 void measurement(bool on)override{call(on?"measure_on":"measure_off");}
 void streaming(bool on)override{call(on?"stream_on":"stream_off");}
 void close()noexcept override{calls.push_back("close");}
};
void require(bool b){if(!b)throw std::runtime_error("assert");}
void rejected(std::function<void()> f){bool bad=false;try{f();}catch(const std::logic_error&){bad=true;}require(bad);}
int main(){using lidar::State;
 Fake d;lidar::Lifecycle l(d);rejected([&]{l.configure();});l.claim();require(l.configure());require(!l.ready());require(l.activate(10));auto e=l.epoch();require(!l.ready());require(!l.frame(e,9.99,10.01));require(!l.frame(e,std::numeric_limits<double>::quiet_NaN(),10.1));require(!l.frame(e-1,10.1,10.1));require(!l.frame(e,10.2,10.1));require(l.frame(e,10.1,10.1));require(l.ready());require(!l.frame(e,10.1,10.2));require(l.stop(true));require(!l.ready()&&l.state()==State::Sleeping);require(!l.frame(e,10.2,10.2));require(l.activate(11));require(l.frame(l.epoch(),11.1,11.1));l.tick(11.5);require(l.state()==State::Fault&&!l.ready());require(l.configure());require(l.state()==State::Inactive&&!l.ready());require(l.release());
 for(auto name:{"open","validate","measure_on","stream_on"}){Fake f;f.fail=name;lidar::Lifecycle x(f);x.claim();bool configured=x.configure();if(configured)x.activate(1);require(x.state()==State::Fault&&!x.ready());}
 for(auto name:{"stream_off","measure_off"}){Fake f;lidar::Lifecycle x(f);x.claim();require(x.configure());require(x.activate(1));f.fail=name;require(!x.stop(true));require(x.state()==State::Fault);bool off=false;for(auto& c:f.calls)off|=c=="measure_off";require(off);}
 Fake f;lidar::Lifecycle x(f);x.claim();x.configure();x.activate(1);x.tick(2.1);require(x.state()==State::Fault);x.configure();x.activate(3);x.ownership_lost();require(!x.ready());rejected([&]{x.activate(4);});
 Fake cold;double acknowledged=12.;lidar::Lifecycle delayed(cold,[&]{return acknowledged;});
 delayed.claim();require(delayed.configure());require(delayed.activate(10.));
 require(!delayed.frame(delayed.epoch(),11.9,12.));
 require(delayed.frame(delayed.epoch(),12.1,12.1));require(delayed.ready());
 require(delayed.stop(true));acknowledged=15.;require(delayed.activate(13.));
 delayed.tick(16.01);require(delayed.state()==State::Fault);
 Fake slow;lidar::Lifecycle warm(slow,{},5.);warm.claim();require(warm.configure());require(warm.activate(20.));
 warm.tick(22.);require(warm.state()==State::Starting&&!warm.ready());
 require(warm.frame(warm.epoch(),22.28,22.28));warm.tick(22.6);require(warm.state()==State::Fault);
 require(warm.configure());require(warm.activate(30.));warm.tick(35.01);require(warm.state()==State::Fault);
 std::cout<<"PASS ownership, configure/activate/frame readiness, sleep/wake, stale/old epoch rejection, timeout, reconnect inactive, stop failure, ownership loss\n";
}
