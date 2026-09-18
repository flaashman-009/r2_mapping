# bags/

建图/定位过程的 rosbag 记录。

```bash
./scripts/record_bag.sh <名字>       # 默认录 120 s，可 Ctrl-C 提前结束
```

录制内容（正则匹配）：

```
/scan*  /odom*  /imu*  /tf  /tf_static  /cmd_vel  /vel_raw  /joint_states  /map*
```

## 提醒

- Jetson 磁盘曾经接近 93% 占用，录之前先 `df -h /`。
- 单次建图 bag 约几十到几百 MB（取决于压缩设置）。
- 确认有 bag 之后再清理 `~/.ros/log`。

