# 节点内部故障报告。
# 业务节点仅负责提供稳定故障码、对象标识和故障现场信息。

# 稳定七位故障码，前三位为系统编码，后四位为系统内错误序号，例如 A410001。
string code

# 同一故障码对应的具体对象标识，例如 left_knee；无具体对象时为空。
string object_id

# 节点在故障发生位置采集的运行时详细信息。
string detail

# 当前这一轮故障首次发生的系统时间；同一故障持续期间保持不变。
builtin_interfaces/Time occurred_at
