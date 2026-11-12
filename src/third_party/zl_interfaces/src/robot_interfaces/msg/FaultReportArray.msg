# 单个故障上报节点当前活动故障的完整快照。

std_msgs/Header header

# 上报节点的完整 ROS 名称，用于区分不同节点实例。
string reporter_id

# 当前活动故障；空数组表示该节点已无活动故障。
FaultReport[] faults
