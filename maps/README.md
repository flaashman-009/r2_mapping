# maps/

保存的地图（PGM + YAML）与 slam_toolbox 序列化结果。

## 命名建议

```
<场地>_<日期>_<版本>.pgm/.yaml
例如：building2f_20260912_v1.yaml
```

## 保存

```bash
./scripts/save_map.sh <名字>
# 等价于调用 slam_toolbox 的 save_map 服务，并额外存一份 map_saver 版本
```

每次保存会产生：

| 文件 | 说明 |
|---|---|
| `<name>.pgm` / `<name>.yaml` | Nav2 可加载的占据栅格地图 |
| `<name>_posegraph.posegraph` / `.data` | slam_toolbox 序列化结果，便于以后续建 |

## 检查

```bash
./scripts/map_eval.sh maps/<name>.yaml
```

## 注意

- `.pgm` 默认在 `.gitignore` 里；要入库用 `git add -f`。
- 地图必须和当次记录（bag、参数、`docs/06_test_log.md`）配套，
  否则出问题没法复现。
