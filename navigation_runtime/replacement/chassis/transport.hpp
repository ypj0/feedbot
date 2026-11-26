#pragma once
#include "lease.hpp"
#include "frame_stream.hpp"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

namespace feedbot_chassis {
// Bounded stream parser: no lifetime allocation, discard one byte on corruption
// to retain overlapping headers. Only validated runtime feedback reaches Lease.
class FeedbackStream {
 FrameStream stream_;
public:
 uint64_t rejected=0, accepted=0;
 template<class Sink> void push(uint8_t byte,Sink sink){
  auto old=stream_.rejected;
  stream_.push(byte,[&](const Frame& frame){
   if(frame[3]==2&&frame[4]==0x60){++accepted;sink(feedback(frame));}
   else ++rejected;
  });
  rejected+=stream_.rejected-old;
 }
};

// Single-owner nonblocking reactor. It never opens a hardware path. The owner
// must establish device exclusion, UART settings and handshake before supplying
// an fd. All calls occur on one dedicated reactor thread, using steady-clock time.
// A blocked/partial output is terminal: flushing or appending a new zero frame
// could splice bytes into an executable malformed command. Host-side software
// cannot promise a physical stop after transport failure.
class Transport {
 using WriteFn=ssize_t (*)(int,const void*,size_t);
 using FeedbackFn=void (*)(const Feedback&,double,void*);
 int fd_;
 WriteFn write_;
 Lease lease_;
 FeedbackStream parser_;
 Frame pending_{};
 size_t offset_=40;
 double next_=0,started_=0,last_=0;
 bool failed_=false;
 FeedbackFn feedback_fn_=nullptr;void* feedback_context_=nullptr;
 void fail(){failed_=true;lease_.trip("serial transport failed");}
public:
 static constexpr double period=.02, write_timeout=.04;
 explicit Transport(int fd,WriteFn writer=::write):fd_(fd),write_(writer){
  if(!writer)throw std::invalid_argument("missing writer");
  int flags=fcntl(fd_,F_GETFL);
  if(flags<0||!(flags&O_NONBLOCK))throw std::invalid_argument("transport requires nonblocking fd");
 }
 Transport(const Transport&)=delete;
 Transport& operator=(const Transport&)=delete;
 bool failed()const{return failed_;}
 bool armed()const{return lease_.armed();}
 bool faulted()const{return lease_.faulted();}
 const char* fault_reason()const{return lease_.fault_reason();}
 uint64_t feedback_count()const{return parser_.accepted;}
 void on_feedback(FeedbackFn fn,void* context){feedback_fn_=fn;feedback_context_=context;}
 const char* arm_blocker(double now,bool stationary)const{return failed_?"serial transport failed":lease_.arm_blocker(now,stationary);}
 bool arm(uint64_t epoch,double now,bool stationary){return !failed_&&lease_.arm(epoch,now,stationary);}
 bool command(uint64_t epoch,uint64_t sequence,double v,double w,double expires,double now){
  return !failed_&&lease_.command(epoch,sequence,v,w,expires,now);
 }
 void stop(const char* reason="safety stop"){lease_.trip(reason);}
 void cancel(){lease_.cancel();}
 bool reset(double now,bool stationary){return !failed_&&lease_.reset(now,stationary);}
 void step(double now){
  if(failed_)return;
  if(!std::isfinite(now)||now<last_){fail();return;}last_=now;
  // RX budget is independent of whether a packet has arrived or is complete.
  uint8_t input[256];
  for(unsigned batch=0;batch<4;batch++){
   ssize_t n=read(fd_,input,sizeof(input));
   if(n>0){for(ssize_t i=0;i<n;i++)parser_.push(input[i],[&](const Feedback& f){lease_.observe(f,now);if(feedback_fn_)feedback_fn_(f,now,feedback_context_);});}
   else if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR))break;
   else {fail();return;}
  }
  Frame target=lease_.tick(now);
  if(offset_<40){
   if(now-started_>=write_timeout){fail();return;}
   if(target!=pending_){
    if(offset_!=0){fail();return;}
    pending_=target; // No bytes sent: replace queued stale target, never extend deadline.
   }
  }else if(now>=next_){pending_=target;offset_=0;started_=now;next_=now+period;}
  if(offset_==40)return;
  ssize_t n=write_(fd_,pending_.data()+offset_,40-offset_);
  if(n>0)offset_+=size_t(n);
  else if(n==0||(errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR))fail();
 }
};
}
