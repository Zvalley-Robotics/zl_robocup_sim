# 单侧灵巧手状态。

# 工具名称。
string name
# 使能状态：0 表示未使能，1 表示已使能。
int32 enable_state
# 状态或错误描述。
string description
# 当前手指压力，单位 g；非触觉版本可能只提供占位数据。
float32[] current_force
# 六个手指执行器的当前位置。
uint16[] actpos
