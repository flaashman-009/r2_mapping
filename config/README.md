# config/

车辆实测参数与待标定占位配置。

## 文件分工

| 文件 | 用途 | 谁读取 |
|---|---|---|
| `r2_vehicle.yaml` | 车辆几何/动力学实测值（只读参考） | 仿真、控制器、标定记录 |
| `lidar_extrinsics.yaml` | LiDAR 外参（**待标定**） | 手动填入 URDF 或 launch 参数 |

**已安装到 ROS 的运行时参数不在这里**，而在
`src/r2_mapping_bringup/config/`（slam_toolbox、amcl、map_server）。
这样避免同一份参数有两个来源。

## 生效方式

`sensors.launch.py` / `mapping.launch.py` 支持直接覆盖 LiDAR 外参：

```bash
ros2 launch r2_mapping_bringup mapping.launch.py \
  publish_lidar_tf:=true \
  lidar_x:=0.10 lidar_y:=0.0 lidar_z:=0.185 lidar_yaw:=0.0
```

⚠️ 前提是 URDF 里同名的 `base_link → laser` 已经注释掉，
否则同一对父子会有两个发布者。

标定流程见 `docs/03_tf_and_extrinsics.md`。

