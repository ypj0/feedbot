# LTME雷达驱动与协议来源核对

已取得厂商LitraTech公开源码：analysis/upstream/ldcp_sdk、analysis/upstream/ltme_node，提交指纹及许可证在analysis/upstream/provenance.json。它们是可读完整参考工程，尚未证明与车机二进制逐字节同版。

SDK在本机编译成静态库成功，未运行设备连接。上游只条件包含Linux/Windows网络头，因此macOS构建通过编译器强制包含netinet/in.h、arpa/inet.h、unistd.h补齐平台声明，未改动算法源码：

```
cmake -S analysis/upstream/ldcp_sdk -B /tmp/feedbot_ldcp_build \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_STANDARD=11 \
  '-DCMAKE_CXX_FLAGS=-include netinet/in.h -include arpa/inet.h -include unistd.h'
cmake --build /tmp/feedbot_ldcp_build -j 2
```

日志：analysis/chassis_stack/ldcp_sdk_build.log。ROS节点本身未在本机ROS Melodic环境编译；不能把SDK构建成功扩大为节点部署完成。

## 已建立的对应关系

车机含ldcp_sdk的Session/Device/NetworkTransport类型信息和scan/startMeasurement、scan/stopMeasurement、scan/startStreaming、scan/stopStreaming等方法字符串，与公开SDK体系对应。SDK使用JSON-RPC2.0控制请求、主连接扫描通知和可选OOB包；主连接传输有checksum封装。协议细节可直接读src/session.cpp、transport.cpp、device.cpp。

车机原0x4dfb0已定位为OOB包校验。最小头16字节，signature=0xffff；校验字段位于12..13，小端；计算前暂时清零字段，用初值0xffff的CRC16，计算后恢复原字段。已恢复lidar_packet.c，2160组完整原指令对照通过，包含长度0..4096、有效/损坏CRC、错误签名及缓冲区恢复；CRC另与Python binascii.crc_hqx核对。

CRC函数验证成功不等于负载结构完整：该校验函数没有核对block_length对应的实际负载大小，不能在新驱动中把它当作完整解析器。负长度不在恢复模型等价性范围内。

上游OOB头还给出frame_index、block_index、block_count、block_length、timestamp和强度位宽/视野标志。公开节点把非零原始距离乘0.002转成LaserScan值；扫描时间也有按主机接收时刻调整的逻辑。这些未经逐函数对照的部分仍标注为上游参考语义，不冒充已验证车机行为。

## 厂家车机相对公开节点的定制

安装节点有公开版本没有的/laser_switch订阅及LaserSwitch_CB。原0x1fae0明确：Int16值等于1则清对象+0x288休眠请求，其他值置休眠请求。它修改请求状态，实际启停由run消费；不是回调内立即完成硬件启停。

requestWakeUpService/requestHibernationService使用trylock后更新同一状态，获取锁失败会返回false；构造函数与run都可重置该字段。不能仅靠一次回调日志判定雷达已经出数据。充电期间上层管理程序的激光启停，也须与这一层状态共同解释。

更新程序update_firmware已只读提取并全函数导出，未执行升级命令。雷达设备内部固件不在这些主机文件中，尚未恢复设备内的发射/采样/滤波实现。
