# MotionAuthority 状态，固定周期发布。
# State / Reason 枚举值必须与 authority.hpp 一致，由 contract/check_contract.py 静态核对。
# 被拒绝是事件而不是状态：last_reject / reject_count 记录它，state 不受影响。

uint8 STATE_IDLE=0
uint8 STATE_ACTIVE=1
uint8 STATE_REVOKED=2
uint8 STATE_FAULT=3

uint8 REASON_NONE=0
uint8 REASON_BUSY=1
uint8 REASON_ALREADY_HELD=2
uint8 REASON_INVALID_SOURCE=3
uint8 REASON_NO_LEASE=4
uint8 REASON_WRONG_LEASE=5
uint8 REASON_WRONG_SOURCE=6
uint8 REASON_BAD_SEQUENCE=7
uint8 REASON_BAD_VALUE=8
uint8 REASON_STALE_STAMP=9
uint8 REASON_FUTURE_STAMP=10
uint8 REASON_EXPIRED=11
uint8 REASON_RELEASED=12
uint8 REASON_FAULT_ACTIVE=13
uint8 REASON_CLOCK_REGRESSION=14
uint8 REASON_BAD_CONFIG=15
uint8 REASON_DOWNSTREAM_FAULT=16

builtin_interfaces/Time stamp
uint8 state
# 取值见 MotionRequest.SOURCE_*；非 ACTIVE 时为 SOURCE_NONE。
uint8 owner
uint64 lease_id
uint8 last_reject
uint8 last_revoke
uint8 fault
# 距最近一次被接受命令的时间；本租约尚无命令为 -1。
int64 input_age_ns
# 非 ACTIVE 为 0。
int64 lease_remaining_ns
# 当前发布到 /wheel/cmd_vel 的速度是否为零。
bool output_zero
uint64 reject_count
uint64 revoke_count
uint64 fault_count
