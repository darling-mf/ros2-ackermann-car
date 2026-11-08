#!/usr/bin/env bash
# 虚拟机 -> 树莓派 一键同步（走 SSH 隧道别名 pi）
# 用法: ~/car_ws/scripts/sync-to-pi.sh            正式同步
#       ~/car_ws/scripts/sync-to-pi.sh --dry-run  只看会传什么
set -e
OPTS="-a --info=progress2"
[ "$1" = "--dry-run" ] && OPTS="$OPTS --dry-run -n"
rsync $OPTS \
  --exclude='build/' --exclude='install/' --exclude='log/' --exclude='.git/' \
  --exclude='README.md' \
  --exclude='src/FAST_LIO/' --exclude='src/FAST-LIO-SAM-QN/' \
  --exclude='*.pcd' \
  ~/car_ws/ pi:~/car_src/