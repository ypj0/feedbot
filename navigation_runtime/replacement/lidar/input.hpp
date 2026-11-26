#pragma once
#include "packet.hpp"
#include <fstream>
#include <cstring>
#include <chrono>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
namespace lidar {
class Input {
 std::ifstream file;int fd=-1;std::chrono::steady_clock::time_point end;
 public:
 Input(const std::string& capture,const std::string& interface,double seconds){
  if(capture.empty()==interface.empty())throw std::runtime_error("choose capture OR passive_interface");
  if(!capture.empty()){file.open(capture,std::ios::binary);if(!file)throw std::runtime_error("capture open");return;}
  if(seconds<=0||seconds>300)throw std::runtime_error("duration must be in (0,300]");
  unsigned index=if_nametoindex(interface.c_str());if(!index)throw std::runtime_error("interface not found");
  // No connect/send calls, no UDP port ownership, no promiscuous membership.
  fd=socket(AF_PACKET,SOCK_RAW,htons(0x0800));if(fd<0)throw std::runtime_error("passive raw socket");
  sockaddr_ll addr{};addr.sll_family=AF_PACKET;addr.sll_protocol=htons(0x0800);addr.sll_ifindex=index;
  int one=1;timeval timeout{0,200000};
  if(bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))||setsockopt(fd,SOL_SOCKET,SO_TIMESTAMPNS,&one,sizeof one)||setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof timeout)){close(fd);fd=-1;throw std::runtime_error("raw socket setup");}
  end=std::chrono::steady_clock::now()+std::chrono::milliseconds(int(seconds*1000));
 }
 ~Input(){if(fd>=0)close(fd);}
 Input(const Input&)=delete;Input& operator=(const Input&)=delete;
 bool next(std::vector<uint8_t>& p,double& stamp){
  if(fd<0){if(file.peek()==EOF)return false;uint32_t n;file.read(reinterpret_cast<char*>(&stamp),8);file.read(reinterpret_cast<char*>(&n),4);if(!file||n>65535)throw std::runtime_error("record header");p.resize(n);file.read(reinterpret_cast<char*>(p.data()),n);if(!file)throw std::runtime_error("record body");return true;}
  while(std::chrono::steady_clock::now()<end){
   p.resize(65535);alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(timespec))]{};iovec iov{p.data(),p.size()};msghdr msg{};msg.msg_iov=&iov;msg.msg_iovlen=1;msg.msg_control=ancillary;msg.msg_controllen=sizeof ancillary;
   ssize_t n=recvmsg(fd,&msg,0);if(n<0){if(errno==EINTR||errno==EAGAIN||errno==EWOULDBLOCK)continue;throw std::runtime_error("raw recvmsg");}
   if(msg.msg_flags&(MSG_TRUNC|MSG_CTRUNC))throw std::runtime_error("truncated raw receive");
   bool found=false;
   for(cmsghdr* c=CMSG_FIRSTHDR(&msg);c;c=CMSG_NXTHDR(&msg,c))if(c->cmsg_level==SOL_SOCKET&&c->cmsg_type==SO_TIMESTAMPNS&&c->cmsg_len>=CMSG_LEN(sizeof(timespec))){timespec ts;std::memcpy(&ts,CMSG_DATA(c),sizeof ts);stamp=ts.tv_sec+ts.tv_nsec*1e-9;found=true;}
   if(!found)throw std::runtime_error("missing kernel receive timestamp");
   p.resize(n);return true;
  }return false;
 }
};
}
