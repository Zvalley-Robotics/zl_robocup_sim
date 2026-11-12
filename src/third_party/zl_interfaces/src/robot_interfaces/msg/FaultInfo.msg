# App 可直接消费的标准故障信息。

# 稳定七位故障码，前三位为系统编码，后四位为系统内错误序号，例如 A410001。
string code

# 故障归属系统的可读名称，由 code 前三位查询系统表获得。
string system_name

# 故障等级：LOW、MEDIUM 或 HIGH。
string level

# 来自故障码字典的稳定中文描述。
string description

# 故障发生位置提供的运行时详细信息。
string detail

# 当前这一轮故障首次发生的东八区时间，格式 YYYY-MM-DD HH:MM:SS.mmm。
string occurred_at
