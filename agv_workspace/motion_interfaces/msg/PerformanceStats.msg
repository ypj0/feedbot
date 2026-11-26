# 高频回调性能统计，每个统计窗口（契约 stats_period_s）发布一次，不在控制周期内发布。

builtin_interfaces/Time stamp
# 产生该统计的节点或回调名称。
string source_name
float64 window_s
uint32 samples
# 窗口内回调间隔超过控制周期的次数。
uint32 overruns
int64 period_ns
int64 callback_mean_ns
int64 callback_p99_ns
int64 callback_max_ns
# 输入消息从其时间戳到被处理的最大年龄。
int64 message_age_max_ns
