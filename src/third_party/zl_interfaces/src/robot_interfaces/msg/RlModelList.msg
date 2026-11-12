# 当前运行平台可用的 RL 模型能力清单。
# robot_rl_manager 在启动完成和清单变化时发布。

# 标准消息头；stamp 为清单生成时间。
std_msgs/Header header

# 当前节点实际用于 artifact 选择的系统架构，例如 x86_64、aarch64。
string runtime_platform
# 启动时要求加载的默认模型名称。
string default_model_name
# 当前平台实际注册成功的模型列表。
RlModelInfo[] models
