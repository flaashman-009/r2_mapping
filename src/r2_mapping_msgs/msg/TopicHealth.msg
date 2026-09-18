# 单个话题的健康状态

string topic              # 话题名，如 /scan
float64 measured_hz       # 实测频率
float64 expected_hz       # 期望频率
float64 age_s             # now - header.stamp，秒
bool ok                   # 是否通过（FAIL 时为 false）
string note               # 说明，例如 "120 帧" / "没有数据"

