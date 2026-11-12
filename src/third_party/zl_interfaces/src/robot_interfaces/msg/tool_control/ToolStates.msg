# 灵巧手状态帧。

# 标准消息头；stamp 为状态采集时间。
std_msgs/Header header
# 对应控制命令的全链路关联 ID。
uint64 trace_id
# 左右灵巧手状态。
ToolState[] states
