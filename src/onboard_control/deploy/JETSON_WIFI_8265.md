<!-- 本文记录 Jetson Linux 39.2.1 指定内核上 Intel 8265 WiFi 的已验证恢复方法。 -->

# Jetson Intel 8265 WiFi 驱动恢复

## 适用范围

本文仅适用于以下已实机验证的组合：

- Jetson Orin NX，aarch64
- Ubuntu 24.04
- Jetson Linux `39.2.1`
- 内核 `6.8.12-1021-tegra`
- Intel Wireless 8265/8275，PCI ID `8086:24fd`
- `backport-iwlwifi-dkms` 版本 `11510-0ubuntu1.1`

不要把本文的 `--force` 命令直接用于其他内核或 Jetson Linux 版本。内核升级后应重新核对 headers、模块 ABI、构建日志和固件加载能力。

## 故障原因

该环境存在两个相互独立的问题。

第一，Jetson 内核配置包含 `CONFIG_CFG80211=m` 和 `CONFIG_MAC80211=m`，但 `CONFIG_IWLWIFI` 未启用。系统已安装匹配的 Tegra headers 和 `backport-iwlwifi-dkms`，DKMS 也已为当前内核成功生成模块，但 Ubuntu 包认为其默认支持上限为 Linux 6.7，因而没有把构建产物安装到 6.8 内核的模块目录。日志中的关键提示为：

```text
in kernel 6.7.0. We will avoid installing for future kernels above 6.7.0.
You may override by specifying --force.
```

第二，当前内核配置为 `# CONFIG_FW_LOADER_COMPRESS is not set`，不能直接加载 `linux-firmware` 提供的 `.ucode.zst` 文件。内核命令行又指定了 `firmware_class.path=/etc/firmware`，但初始系统中该目录不存在。因此即使驱动模块加载成功，仍会因找不到未压缩固件而无法创建设备。

## 前提检查

先确认系统和硬件完全匹配本文范围：

```bash
uname -r
dpkg-query -W nvidia-l4t-kernel nvidia-l4t-kernel-headers \
  backport-iwlwifi-dkms dkms zstd
lspci -nnk -s 0001:01:00.0
```

预期内核为 `6.8.12-1021-tegra`，`nvidia-l4t-kernel` 为 `39.2.1` 对应版本，无线设备为 `8086:24fd`。

确认匹配 headers 已准备好，并包含当前内核的符号表：

```bash
readlink -f /lib/modules/6.8.12-1021-tegra/build
test -s /lib/modules/6.8.12-1021-tegra/build/Module.symvers
test -s /lib/modules/6.8.12-1021-tegra/build/.config
```

确认 DKMS 已成功构建六个模块：

```bash
dkms status -m backport-iwlwifi -v 11510
ls -l /var/lib/dkms/backport-iwlwifi/11510/6.8.12-1021-tegra/aarch64/module/
grep -Ei 'error:|fatal:' \
  /var/lib/dkms/backport-iwlwifi/11510/6.8.12-1021-tegra/aarch64/log/make.log
```

构建目录中应已有 `iwlwifi.ko`、`iwlmvm.ko`、`iwlwifi-compat.ko`、`iwlxvt.ko`、`cfg80211.ko` 和 `mac80211.ko`，且日志中不应有编译错误。已有这些构建产物时无需重新编译。

确认固件文件和内核压缩配置：

```bash
ls -l /lib/firmware/iwlwifi-8265-34.ucode.zst \
  /lib/firmware/iwlwifi-8265-36.ucode.zst
zcat /proc/config.gz | grep -E '^CONFIG_FW_LOADER|^# CONFIG_FW_LOADER_COMPRESS'
cat /sys/module/firmware_class/parameters/path
```

## 安装已有 DKMS 构建

仅在上述版本和构建产物均确认匹配后执行：

```bash
sudo dkms install --force \
  -m backport-iwlwifi \
  -v 11510 \
  -k 6.8.12-1021-tegra
sudo depmod -a 6.8.12-1021-tegra
```

确认模块已安装到当前内核并能被解析：

```bash
dkms status -m backport-iwlwifi -v 11510 -k 6.8.12-1021-tegra
modinfo -F filename iwlwifi
modinfo -F filename iwlmvm
```

预期模块位于：

```text
/lib/modules/6.8.12-1021-tegra/updates/dkms/
```

该 DKMS 包同时提供匹配版本的 `cfg80211` 和 `mac80211`。不要只复制 `iwlwifi.ko` 或 `iwlmvm.ko`。

## 准备未压缩固件

从系统已有的压缩固件生成未压缩副本，不需要联网下载：

```bash
sudo install -d -m 0755 /etc/firmware

for version in 34 36; do
  source_file="/lib/firmware/iwlwifi-8265-${version}.ucode.zst"
  target_file="/etc/firmware/iwlwifi-8265-${version}.ucode"
  zstd -dc "${source_file}" | sudo tee "${target_file}.tmp" >/dev/null
  sudo chmod 0644 "${target_file}.tmp"
  sudo mv "${target_file}.tmp" "${target_file}"
done
```

加载驱动：

```bash
sudo modprobe iwlwifi
```

首次加载时可能出现模块签名验证警告并使内核带 taint 标志。已验证内核没有启用 `CONFIG_MODULE_SIG_FORCE`，因此该警告不会阻止模块加载。

## 验证

```bash
lspci -nnk -s 0001:01:00.0
ip -br link
rfkill list all
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
nmcli device wifi list --rescan yes
sudo dmesg | grep -Ei 'iwlwifi|iwlmvm|firmware|wlan' | tail -n 100
```

本次实机验证结果为：

- PCI 设备显示 `Kernel driver in use: iwlwifi`。
- 驱动加载 `36.ca7b901d.0 8265-36.ucode`。
- NetworkManager 识别无线接口 `wlP1p1s0`。
- rfkill 的软、硬阻止均为 `no`。
- 2.4 GHz 和 5 GHz WiFi 扫描均成功。
- 未连接任何 WiFi 网络。

2026-09-16 完成系统升级后已重启验证：同一内核自动加载 iwlwifi，无线接口正常出现，2.4/5 GHz 热点扫描成功，软硬阻止均为 no。未配置 WiFi 密码，因此未进行热点连接或无线互联网测试。
