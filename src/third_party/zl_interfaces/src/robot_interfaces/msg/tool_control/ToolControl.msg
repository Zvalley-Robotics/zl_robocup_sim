# 单侧灵巧手控制目标。

# 工具名称，当前驱动支持 Left 和 Right。
string name
# 六个手指执行器的目标压力/电流，统一范围由底层 SDK 定义。
int16[] tar_force
# 六个手指执行器的目标位置，统一范围 0~1000。
uint16[] tar_pos
# 六个手指执行器的动作时间，单位 ms。
uint16[] tar_duration
