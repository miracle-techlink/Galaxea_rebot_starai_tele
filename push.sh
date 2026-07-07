#!/usr/bin/env bash
# 可靠推送:覆盖全局 ghfast 只读镜像改写 + 用 gh 凭据,直连 github 推送。
# 用法: bash push.sh            (推 HEAD 到 origin)
set -e
cd "$(dirname "$0")"
TOKEN="$(gh auth token 2>/dev/null)"
git -c "url.https://ghfast.top/https://github.com/.insteadOf=x-disabled://" \
    -c credential.helper="!f(){ echo username=miracle-techlink; echo password=$TOKEN; };f" \
    push origin "${1:-HEAD}"
