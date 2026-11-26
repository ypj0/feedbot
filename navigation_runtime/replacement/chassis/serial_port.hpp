#pragma once
#include "protocol.hpp"
#include "frame_stream.hpp"
#include <cerrno>
#include <chrono>
#include <cstdlib>
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
namespace feedbot_chassis {
inline double steady_seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline void require_serial(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
inline void audit_serial_owners(const std::string& path){
#ifdef __linux__
 struct stat wanted{};require_serial(stat(path.c_str(),&wanted)==0&&S_ISCHR(wanted.st_mode),"invalid serial device");
 DIR* proc=opendir("/proc");require_serial(proc,"cannot audit device owners");bool conflict=false,incomplete=false;
 while(auto e=readdir(proc)){
  char* end=nullptr;long pid=strtol(e->d_name,&end,10);if(!pid||*end||pid==getpid())continue;
  std::string dir=std::string("/proc/")+e->d_name+"/fd";DIR* fds=opendir(dir.c_str());
  if(!fds){if(errno!=ENOENT&&errno!=ESRCH)incomplete=true;continue;}
  while(auto f=readdir(fds)){
   if(f->d_name[0]=='.')continue;
   struct stat s{};
   if(stat((dir+"/"+f->d_name).c_str(),&s)==0){if(S_ISCHR(s.st_mode)&&s.st_rdev==wanted.st_rdev)conflict=true;}
   else if(errno!=ENOENT&&errno!=ESRCH)incomplete=true;
  }
  closedir(fds);
 }
 closedir(proc);require_serial(!conflict,"serial device already owned");require_serial(!incomplete,"incomplete owner audit");
#else
 (void)path;throw std::runtime_error("physical owner audit requires Linux");
#endif
}
class SerialPort {
 int fd_=-1;bool exclusive_=false,configured_=false;termios saved_{};
public:
 explicit SerialPort(const std::string& path,bool fixture=false){
  if(fixture)require_serial(path.find("/dev/pts/")==0||path.find("/dev/ttys")==0,"fixture must use PTY");
  else audit_serial_owners(path);
  fd_=open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);require_serial(fd_>=0,"open serial failed");
  try{
   require_serial(ioctl(fd_,TIOCEXCL)==0,"exclusive serial failed");exclusive_=true;
   if(!fixture)audit_serial_owners(path);
   require_serial(tcgetattr(fd_,&saved_)==0,"get termios failed");auto raw=saved_;cfmakeraw(&raw);
   cfsetispeed(&raw,B115200);cfsetospeed(&raw,B115200);raw.c_cflag|=CLOCAL|CREAD;
   raw.c_cflag&=~(CSTOPB|PARENB|CRTSCTS);raw.c_cc[VMIN]=1;raw.c_cc[VTIME]=0;
   require_serial(tcsetattr(fd_,TCSANOW,&raw)==0,"configure serial failed");configured_=true;
   require_serial(tcflush(fd_,TCIFLUSH)==0,"flush input failed");
  }catch(...){close_port();throw;}
 }
 SerialPort(const SerialPort&)=delete;SerialPort& operator=(const SerialPort&)=delete;
 ~SerialPort(){close_port();}
 int fd()const{return fd_;}
 void close_port(){if(fd_>=0){if(configured_)tcsetattr(fd_,TCSANOW,&saved_);if(exclusive_)ioctl(fd_,TIOCNXCL);close(fd_);fd_=-1;}}
 void handshake(){
  Frame h{};h[0]=h[1]=127;h[2]=40;h[3]=1;h[38]=13;h[39]=10;checksum(h);
  auto start=steady_seconds();size_t sent=0;
  while(sent<h.size()){
   require_serial(steady_seconds()-start<.04,"handshake write deadline");
   auto n=write(fd_,h.data()+sent,h.size()-sent);if(n>0)sent+=size_t(n);
   else require_serial(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR),"handshake write failed");
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  FrameStream parser;bool ready=false;
  while(!ready){
   require_serial(steady_seconds()-start<1.,"handshake read deadline");uint8_t b[256];auto n=read(fd_,b,sizeof(b));
   if(n>0)for(ssize_t i=0;i<n;i++)parser.push(b[i],[&](const Frame& f){
    uint32_t baud=(uint32_t(f[5])<<24)|(uint32_t(f[6])<<16)|(uint32_t(f[7])<<8)|f[8];
    require_serial(f[3]==1&&f[4]==0x60&&baud==115200,"invalid handshake response");ready=true;
   });
   else require_serial(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR),"handshake read failed");
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
 }
};
}
