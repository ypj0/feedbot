# 运动请求。只有持有有效租约的来源会被接受，租约号来自 AcquireLease。
# 枚举值必须与 motion_authority/include/feedbot_motion_authority/authority.hpp 的 Source 一致，
# 由 agv_workspace/contract/check_contract.py 静态核对。

uint8 SOURCE_NONE=0
uint8 SOURCE_MANUAL=1
uint8 SOURCE_NAV2=2
uint8 SOURCE_DOCK=3

# 发送方时间戳，系统时钟（use_sim_time=false）。过旧或过新的请求会被拒绝。
builtin_interfaces/Time stamp
uint8 source
uint64 lease_id
# 每个租约内严格递增，从 1 开始；允许跳号，不允许重复或倒退。
uint64 seq
# 差速底盘只使用 linear.x 与 angular.z，其余分量必须为 0。
geometry_msgs/Twist twist
