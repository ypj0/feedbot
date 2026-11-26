# 串口独占审计的容器权限

冻结的 SerialPort 在打开串口前后扫描 `/proc/PID/fd`。仅映射串口设备不够：容器必须能看到宿主 PID，并完成这些描述符的读取审计。任何访问拒绝仍报错，不跳过、不改为容器内局部审计。

部署组合：

```sh
apparmor_parser -a feedbot-serial-audit.apparmor
docker run --pid host --cap-add SYS_PTRACE \
  --security-opt apparmor=feedbot-serial-audit ...
```

策略安装路径为 `/etc/apparmor.d/feedbot-serial-audit`；首次加载使用 `-a`，已加载策略的明确升级使用 `-r`，不是失败重试。启动前确认 `/sys/kernel/security/apparmor/profiles` 中为 enforce。Docker 默认 seccomp、设备白名单及其他容器约束保持；不使用 privileged 或 apparmor=unconfined，不修改 docker-default。

策略基于现场 Docker 27.5.1 dockerd 内置 AppArmor 默认模板（Moby `profiles/apparmor/template.go`，Apache-2.0），保留文件、挂载、sys/proc 写限制和容器内信号/ptrace 规则，只新增跨 profile 的 ptrace(read)。模板由现场二进制提取核对，未凭空重建原规则。原模板中 daemon=unconfined 与固定 unconfined 的重复 receive 规则合为一条。

SYS_PTRACE 本身不能绕过 docker-default 的 AppArmor 拒绝。专用策略允许跨进程只读检查，不允许跨 profile 的 ptrace(trace) 或发送信号；但内核 ptrace(read) 权限粒度大于“仅枚举 fd”，不能宣称它只允许读取 fd 元数据。仅用于可信的串口驱动容器，不能用于无关服务。

只读前置检查使用 `acceptance/runs/20261123_chassis_zero_33/audit_proc.py`，容器不映射设备、network=none，返回非零就停止部署。该检查不打开串口、不中断原厂服务，也不替代激活后的独占与恢复验收。
