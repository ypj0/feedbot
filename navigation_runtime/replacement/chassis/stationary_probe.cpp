// Bounded maintenance probe. No arm, velocity or action command API exists here.
#include "protocol.hpp"
#include "frame_stream.hpp"
#include <cstdlib>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#ifdef __linux__
#include <dirent.h>
#endif
using namespace feedbot_chassis;
namespace {
volatile sig_atomic_t interrupted=0;
void on_signal(int){interrupted=1;}
double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
struct Port {
 int fd=-1;bool exclusive=false;termios original{};bool configured=false;
 ~Port(){if(fd>=0){if(configured)tcsetattr(fd,TCSANOW,&original);if(exclusive)ioctl(fd,TIOCNXCL);close(fd);}}
};
// TIOCEXCL does not evict already open descriptors, and root can bypass it.
// The systemd worker separately stops the factory service. Inspect aliases by
// device number, not string path. Failure to inspect another process is refusal.
void no_other_owners(const char* path){
#ifdef __linux__
 struct stat wanted{};require(stat(path,&wanted)==0&&S_ISCHR(wanted.st_mode),"not a character device");
 DIR* proc=opendir("/proc");require(proc,"cannot inspect owners");
 bool conflict=false,unreadable=false;
 while(auto entry=readdir(proc)){
  char* end=nullptr;long pid=strtol(entry->d_name,&end,10);
  if(!pid||*end||pid==getpid())continue;
  std::string dir=std::string("/proc/")+entry->d_name+"/fd";
  DIR* fds=opendir(dir.c_str());
  if(!fds){if(errno!=ENOENT&&errno!=ESRCH)unreadable=true;continue;}
  while(auto f=readdir(fds)){
   if(f->d_name[0]=='.')continue;
   struct stat s{};
   if(stat((dir+"/"+f->d_name).c_str(),&s)==0){
    if(S_ISCHR(s.st_mode)&&s.st_rdev==wanted.st_rdev)conflict=true;
   }else if(errno!=ENOENT&&errno!=ESRCH)unreadable=true;
  }
  closedir(fds);
 }
 closedir(proc);require(!conflict,"serial device already owned");require(!unreadable,"owner audit incomplete");
#else
 (void)path;throw std::runtime_error("hardware ownership audit requires Linux");
#endif
}
// Deadline bounds include partial writes. Never tcdrain() or flush an incomplete
// command into a new frame. A failed exchange exits, retaining physical E-stop.
void send(int fd,const Frame& f){
 size_t offset=0;double end=now()+.04;
 while(offset<f.size()){
  require(!interrupted&&now()<end,"write deadline/interruption");
  ssize_t n=write(fd,f.data()+offset,f.size()-offset);
  if(n>0)offset+=size_t(n);
  else {require(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR),"write error");std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 }
}
struct Stream {
 FrameStream parser;
 template<class Sink>void read_once(int fd,Sink sink){
  uint8_t bytes[256];ssize_t size=read(fd,bytes,sizeof(bytes));
  if(size<0){require(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR,"read error");return;}
  require(size!=0,"serial disconnected");
  for(ssize_t i=0;i<size;i++)parser.push(bytes[i],sink);
 }
};
}
int main(int argc,char** argv){
 try{
  require(argc==3,"usage: stationary_probe --pty|--stationary-maintenance DEVICE");
  bool fixture=std::string(argv[1])=="--pty";
  require(fixture||std::string(argv[1])=="--stationary-maintenance","invalid mode");
  std::string path=argv[2];
  if(fixture)require(path.find("/dev/pts/")==0||path.find("/dev/ttys")==0,"fixture requires PTY path");
  else no_other_owners(argv[2]);
  signal(SIGTERM,on_signal);signal(SIGINT,on_signal);signal(SIGPIPE,SIG_IGN);
  Port p;p.fd=open(argv[2],O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);require(p.fd>=0,"open failed");
  require(ioctl(p.fd,TIOCEXCL)==0,"exclusive open failed");p.exclusive=true;
  if(!fixture)no_other_owners(argv[2]);
  require(tcgetattr(p.fd,&p.original)==0,"get termios failed");
  termios raw=p.original;cfmakeraw(&raw);cfsetispeed(&raw,B115200);cfsetospeed(&raw,B115200);
  raw.c_cflag|=CLOCAL|CREAD;raw.c_cflag&=~(CSTOPB|PARENB|CRTSCTS);raw.c_cc[VMIN]=1;raw.c_cc[VTIME]=0;
  require(tcsetattr(p.fd,TCSANOW,&raw)==0,"configure termios failed");p.configured=true;
  // No received bytes from the former owner are accepted as fresh evidence.
  require(tcflush(p.fd,TCIFLUSH)==0,"input flush failed");
  Frame hello{};hello[0]=hello[1]=127;hello[2]=40;hello[3]=1;hello[38]=13;hello[39]=10;checksum(hello);
  send(p.fd,hello);Stream stream;bool handshake=false;double start=now();
  while(!handshake){
   require(!interrupted&&now()-start<1.,"handshake timeout/interruption");
   stream.read_once(p.fd,[&](const Frame& f){
    require(f[3]==1&&f[4]==0x60,"unexpected handshake response");
    uint32_t baud=(uint32_t(f[5])<<24)|(uint32_t(f[6])<<16)|(uint32_t(f[7])<<8)|f[8];
    require(baud==115200,"MCU baud mismatch");handshake=true;
   });
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cout<<"{\"event\":\"handshake\",\"baud\":115200}"<<std::endl;
  unsigned tx=0,rx=0;double begin=now(),next=begin,last=begin,max_gap=0;bool seen=false;
  while(now()-begin<(fixture?.5:10.)){
   require(!interrupted,"interrupted");double at=now();
   stream.read_once(p.fd,[&](const Frame& frame){
    auto f=feedback(frame);
    require(f.emergency==1&&f.v_raw==0&&f.w_raw==0&&f.version==441&&f.initialized==1,"maintenance feedback condition failed");
    double stamp=now();if(seen)max_gap=std::max(max_gap,stamp-last);last=stamp;seen=true;++rx;
   });
   require(now()-last<.2,"runtime feedback stale");
   if(at>=next){
    int queued=0;require(ioctl(p.fd,TIOCOUTQ,&queued)==0&&queued==0,"output backlog");
    send(p.fd,kReviewedStationary);++tx;next=at+.02;
   }
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(seen&&rx>0,"no runtime feedback");
  std::cout<<"{\"event\":\"complete\",\"tx\":"<<tx<<",\"rx\":"<<rx<<",\"max_feedback_gap_s\":"<<max_gap<<",\"motion_commands\":0}"<<std::endl;
  return 0;
 }catch(const std::exception& e){std::cerr<<"stationary_probe: "<<e.what()<<std::endl;return 1;}
}
