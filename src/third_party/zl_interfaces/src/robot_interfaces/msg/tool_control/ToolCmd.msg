# 灵巧手控制命令帧。

# 标准消息头；stamp 为命令生成时间。
std_msgs/Header header
# 全链路关联 ID，由命令源生成并由状态反馈透传。
uint64 trace_id
# 左右灵巧手控制目标。
ToolControl[] cmds
