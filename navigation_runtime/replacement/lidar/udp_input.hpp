#pragma once
#include "packet.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
namespace lidar {
class UdpInput {
 int fd=-1;in_addr expected{};
 public:
 UdpInput(const std::string& ip,uint16_t port){
  if(inet_pton(AF_INET,ip.c_str(),&expected)!=1)throw std::runtime_error("device IPv4");
  fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);if(fd<0)throw std::runtime_error("UDP socket");
  sockaddr_in local{};local.sin_family=AF_INET;local.sin_port=htons(port);local.sin_addr.s_addr=INADDR_ANY;int one=1;
  // No SO_REUSEADDR: a second driver must fail to acquire this port.
  if(bind(fd,reinterpret_cast<sockaddr*>(&local),sizeof local)||setsockopt(fd,SOL_SOCKET,SO_TIMESTAMPNS,&one,sizeof one)){close(fd);fd=-1;throw std::runtime_error("exclusive UDP bind/timestamp");}
 }
 ~UdpInput(){if(fd>=0)close(fd);}
 UdpInput(const UdpInput&)=delete;UdpInput& operator=(const UdpInput&)=delete;
 bool read(Block& block,double& stamp){
  uint8_t bytes[65536];alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(timespec))]{};sockaddr_in peer{};iovec iov{bytes,sizeof bytes};msghdr m{};m.msg_name=&peer;m.msg_namelen=sizeof peer;m.msg_iov=&iov;m.msg_iovlen=1;m.msg_control=ancillary;m.msg_controllen=sizeof ancillary;
  auto n=recvmsg(fd,&m,0);if(n<0){if(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)return false;throw std::runtime_error("UDP receive");}
  if(peer.sin_addr.s_addr!=expected.s_addr)throw std::runtime_error("unexpected UDP sender");
  if(m.msg_flags&(MSG_TRUNC|MSG_CTRUNC))throw std::runtime_error("UDP truncated");
  bool valid=false;for(auto* c=CMSG_FIRSTHDR(&m);c;c=CMSG_NXTHDR(&m,c))if(c->cmsg_level==SOL_SOCKET&&c->cmsg_type==SO_TIMESTAMPNS&&c->cmsg_len>=CMSG_LEN(sizeof(timespec))){timespec t;std::memcpy(&t,CMSG_DATA(c),sizeof t);stamp=t.tv_sec+t.tv_nsec*1e-9;valid=true;}
  if(!valid)throw std::runtime_error("UDP timestamp missing");block=decode(bytes,n);return true;
 }
 void drain(){uint8_t bytes[65536];for(int i=0;i<512;i++)if(recv(fd,bytes,sizeof bytes,0)<0)return;throw std::runtime_error("UDP drain overflow");}
};
}
