<!-- Preserve pre-trim device configurations without changing the active selection. -->
# 配置备份

`20261019-flight-trim/` 保存本机裁剪前的四份 YAML 和兼容链接，`SHA256SUMS` 可核验备份内容。
开发机和飞机各自先备份，再修改各自配置；飞机的地图路径和标定不得由开发机备份覆盖。
恢复所需文件后重启 Odin 即可；`original.yaml` 本次保持不变。
