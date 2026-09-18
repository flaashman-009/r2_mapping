# 一次完整自检的汇总结果

builtin_interfaces/Time stamp

bool ok                          # 全部关键项通过
bool tf_tree_complete            # TF 链路完整
float64 max_stamp_skew_s         # 传感器时间戳最大互差，秒

TopicHealth[] topics
TfHealth[] transforms

string[] warnings                # 包含警告与失败项的可读描述

