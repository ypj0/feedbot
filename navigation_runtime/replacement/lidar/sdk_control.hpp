#pragma once
#include "lifecycle.hpp"
#include <ldcp/device.h>
#include <ldcp/location.h>
#include <arpa/inet.h>
#include <memory>
namespace lidar {
class SdkControl final:public Control {
 std::string expected_model;std::unique_ptr<ldcp_sdk::Device> device;in_addr_t target;in_port_t data_port;
 static void check(ldcp_sdk::error_t e){if(e!=ldcp_sdk::no_error)throw std::runtime_error("LDCP failure code "+std::to_string(int(e)));}
 public:
 SdkControl(const std::string& ip,const std::string& expected_target,uint16_t port=2105,uint16_t udp_port=2105,const std::string& model="LT-R2"):expected_model(model),data_port(htons(udp_port)){
  in_addr address{},destination{};if(inet_pton(AF_INET,ip.c_str(),&address)!=1||inet_pton(AF_INET,expected_target.c_str(),&destination)!=1)throw std::runtime_error("IPv4");target=destination.s_addr;
  ldcp_sdk::NetworkLocation location(address.s_addr,htons(port));device.reset(new ldcp_sdk::Device(location));device->setTimeout(500);
 }
 void open() override {
  // Control-only session: Device::open also binds the UDP receiver. Keep data plane separate.
  check(device->DeviceBase::open());
 }
 void validate() override {
  std::string model;bool oob=false;in_addr_t destination=0;in_port_t port=0;
  check(device->queryModel(model));check(device->isOobEnabled(oob));check(device->getOobTargetAddress(destination));check(device->getOobTargetPort(port));
  if(model!=expected_model||!oob||destination!=target||port!=data_port)throw std::runtime_error("device identity/UDP configuration mismatch: model="+model+" oob="+std::to_string(oob)+" target="+std::to_string(ntohl(destination))+" port="+std::to_string(ntohs(port)));
 }
 void measurement(bool on)override{if(!device->isOpened())throw std::runtime_error("control disconnected");check(on?device->startMeasurement():device->stopMeasurement());}
 void streaming(bool on)override{if(!device->isOpened())throw std::runtime_error("control disconnected");check(on?device->startStreaming():device->stopStreaming());}
 void close()noexcept override{try{device->close();}catch(...){}}
};
}
