#pragma once
#include "protocol.hpp"
#include <algorithm>
namespace feedbot_chassis {
// Common fixed-memory framing for handshake and runtime packets.
class FrameStream {
 Frame bytes_{};size_t count_=0;
public:
 uint64_t rejected=0,accepted=0;
 template<class Sink>void push(uint8_t byte,Sink sink){
  bytes_[count_++]=byte;
  while(count_&&(bytes_[0]!=127||(count_>1&&bytes_[1]!=127)||(count_>2&&bytes_[2]!=40))){
   std::move(bytes_.begin()+1,bytes_.begin()+count_,bytes_.begin());--count_;++rejected;
  }
  if(count_!=40)return;
  if(valid(bytes_)){Frame complete=bytes_;count_=0;++accepted;sink(complete);}
  else {std::move(bytes_.begin()+1,bytes_.end(),bytes_.begin());--count_;++rejected;}
 }
};
}
