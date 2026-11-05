#!/usr/bin/env bash
# 라즈베리파이(Ubuntu 24.04)에서 연결된 장치가 보이는지 한 번에 확인
#   실행:  bash codes/check_connections.sh
# 값이 맞는지는 setuptest.py(센서) / motor_test.py(모터) 로 확인한다.

ok()   { echo "  [OK]   $*"; }
warn() { echo "  [확인] $*"; }

echo "== 1. USB 시리얼 장치 (ESP32, 레이더용 CP2102 어댑터) =="
if ls /dev/serial/by-id/* >/dev/null 2>&1; then
  for p in /dev/serial/by-id/*; do ok "$(basename "$p") -> $(readlink -f "$p")"; done
  n=$(ls /dev/serial/by-id/ | wc -l)
  [ "$n" -lt 2 ] && warn "USB 시리얼이 ${n}개뿐. ESP32 와 CP2102 둘 다 꽂으면 2개가 보여야 함"
else
  warn "USB 시리얼 장치 없음 -> ESP32/CP2102 USB 케이블 확인"
fi

echo
echo "== 2. GPS 용 UART (GPIO14/15) =="
if [ -e /dev/serial0 ]; then
  ok "/dev/serial0 -> $(readlink -f /dev/serial0)"
elif [ -e /dev/ttyS0 ]; then
  ok "/dev/ttyS0 있음 (serial0 링크는 없음 -> setuptest.py 에 --gnss-port /dev/ttyS0)"
else
  warn "GPIO UART 장치 없음 -> /boot/firmware/config.txt 에 enable_uart=1"
fi
if grep -q "console=serial0" /boot/firmware/cmdline.txt 2>/dev/null; then
  warn "시리얼 콘솔이 GPIO UART 를 쓰고 있음 -> GPS 값이 안 읽힘 (docs/연결확인_가이드.md 2단계)"
else
  ok "시리얼 콘솔 꺼져 있음"
fi

echo
echo "== 3. I2C (GY-85) =="
if [ ! -e /dev/i2c-1 ]; then
  warn "/dev/i2c-1 없음 -> /boot/firmware/config.txt 에 dtparam=i2c_arm=on"
elif ! command -v i2cdetect >/dev/null; then
  warn "i2cdetect 없음 -> sudo apt install -y i2c-tools"
else
  scan=$(sudo i2cdetect -y 1)
  echo "$scan"
  for addr in 53 68; do
    if echo "$scan" | grep -qw "$addr"; then ok "0x$addr 응답"; else warn "0x$addr 없음 -> GY-85 SDA/SCL/3.3V/GND 확인"; fi
  done
fi

echo
echo "== 4. 카메라 =="
if ls /dev/video* >/dev/null 2>&1; then ok "$(ls /dev/video* | tr '\n' ' ')"; else warn "/dev/video* 없음 -> 리본케이블 방향/커넥터 확인"; fi

echo
echo "== 5. ROS 2 =="
if ls /opt/ros >/dev/null 2>&1; then ok "설치된 배포판: $(ls /opt/ros | tr '\n' ' ')"; else warn "/opt/ros 없음"; fi

echo
echo "== 6. 권한 =="
if id -nG | grep -qw dialout; then ok "dialout 그룹 (시리얼 포트 사용 가능)"; else warn "dialout 그룹 아님 -> sudo usermod -aG dialout \$USER 후 재로그인"; fi
