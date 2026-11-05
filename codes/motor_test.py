#!/usr/bin/env python3
"""
라즈베리파이 -> ESP32-S3 -> MDD3A 모터 + 엔코더 테스트.

ESP32 에 esp32/motor_encoder_test/motor_encoder_test.ino 를 먼저 업로드한 뒤 실행:

    python3 codes/motor_test.py              # ESP32 포트 자동 탐색
    python3 codes/motor_test.py --port /dev/ttyUSB0
    python3 codes/motor_test.py --speed 150  # 속도 바꾸기 (0~255)

확인하는 것
  1. 라즈베리파이 <-> ESP32 통신 (체크섬 포함 상태 메시지 수신)
  2. 좌/우 모터가 명령대로 도는지 (PWM 값)
  3. 엔코더 카운트가 바뀌는지, 방향이 맞는지
  4. 명령을 300ms 끊으면 ESP32 가 스스로 멈추는지 (워치독)

바퀴를 공중에 띄우고 실행할 것.
"""

from __future__ import annotations

import argparse
import glob
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

try:
    import serial
except ImportError:
    sys.exit("pyserial 이 없습니다:  sudo apt install -y python3-serial")


BAUD = 115200


# ---------- 프로토콜 ----------
def checksum(body: str) -> int:
    c = 0
    for ch in body.encode("ascii"):
        c ^= ch
    return c


def make_line(body: str) -> bytes:
    return "${}*{:02X}\n".format(body, checksum(body)).encode("ascii")


def parse_line(raw: bytes) -> Optional[List[str]]:
    """'$E,...*HH' -> ['E', ...]. 체크섬이 틀리면 None."""
    try:
        line = raw.decode("ascii").strip()
    except UnicodeDecodeError:
        return None
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, cs = line[1:].rpartition("*")
    try:
        if int(cs, 16) != checksum(body):
            return None
    except ValueError:
        return None
    return body.split(",")


@dataclass
class Status:
    ms: int
    enc_left: int
    enc_right: int
    pwm_left: int
    pwm_right: int
    watchdog: bool
    checksum_errors: int


def parse_status(fields: List[str]) -> Optional[Status]:
    if len(fields) != 8 or fields[0] != "E":
        return None
    try:
        v = [int(x) for x in fields[1:]]
    except ValueError:
        return None
    return Status(v[0], v[1], v[2], v[3], v[4], v[5] == 1, v[6])


# ---------- 포트 찾기 ----------
def candidate_ports() -> List[str]:
    ports = sorted(glob.glob("/dev/serial/by-id/*"))
    ports += sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    seen, out = set(), []
    for p in ports:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def looks_like_esp32(port: str, seconds: float = 1.5) -> bool:
    """115200 으로 열어서 체크섬 맞는 '$E,' 줄이 오면 ESP32 로 판단 (레이더는 256000 바이너리라 안 걸림)."""
    try:
        with serial.Serial(port, BAUD, timeout=0.2) as s:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                fields = parse_line(s.readline())
                if fields and parse_status(fields):
                    return True
    except (serial.SerialException, OSError):
        return False
    return False


def find_esp32() -> Optional[str]:
    for port in candidate_ports():
        print("  포트 확인 중: {}".format(port))
        if looks_like_esp32(port):
            return port
    return None


# ---------- 테스트 ----------
class Esp32Link:
    def __init__(self, port: str) -> None:
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self.status: Optional[Status] = None
        self.bad_lines = 0

    def send_motor(self, left: int, right: int) -> None:
        self.ser.write(make_line("M,{},{}".format(int(left), int(right))))

    def reset_encoders(self) -> None:
        self.ser.write(make_line("R"))

    def poll(self) -> None:
        while self.ser.in_waiting:
            raw = self.ser.readline()
            fields = parse_line(raw)
            if fields is None:
                if raw.strip():
                    self.bad_lines += 1
                continue
            if fields[0] == "E":
                st = parse_status(fields)
                if st:
                    self.status = st
            elif fields[0] == "ERR":
                print("  [ESP32 오류] {}".format(",".join(fields[1:])))

    def close(self) -> None:
        try:
            for _ in range(3):
                self.send_motor(0, 0)
                time.sleep(0.05)
        finally:
            self.ser.close()


def run_step(link: Esp32Link, name: str, left: int, right: int, seconds: float, settle: float = 0.4) -> dict:
    """명령을 10Hz 로 보내면서 엔코더 변화를 기록.
    처음 settle 초는 가속/감속 시간이라 변화량 계산에서 뺀다."""
    print("\n[{}]  명령 좌={} 우={}  ({:.1f}초)".format(name, left, right, seconds))
    t_begin = time.monotonic()
    t_end = t_begin + max(seconds, settle + 0.5)
    start: Optional[Status] = None
    next_send = 0.0
    next_print = t_begin + 0.5
    while time.monotonic() < t_end:
        now = time.monotonic()
        if start is None and now - t_begin >= settle:
            link.poll()
            start = link.status
        if now >= next_send:
            link.send_motor(left, right)
            next_send = now + 0.1           # 300ms 워치독보다 충분히 짧게
        link.poll()
        if now >= next_print and link.status:
            s = link.status
            print("    엔코더 좌={:>8} 우={:>8} | PWM 좌={:>4} 우={:>4} | 워치독={}".format(
                s.enc_left, s.enc_right, s.pwm_left, s.pwm_right, "정지" if s.watchdog else "정상"))
            next_print = now + 0.5
        time.sleep(0.01)
    link.poll()
    end = link.status
    d_left = (end.enc_left - start.enc_left) if (start and end) else 0
    d_right = (end.enc_right - start.enc_right) if (start and end) else 0
    print("    -> 이 구간 엔코더 변화: 좌 {:+d}, 우 {:+d}".format(d_left, d_right))
    return {"name": name, "left": left, "right": right, "d_left": d_left, "d_right": d_right}


def judge(cmd: int, delta: int, side: str) -> str:
    if cmd == 0:
        return "OK (멈춤)" if abs(delta) < 20 else "확인 필요: 명령 0 인데 {} 카운트 움직임".format(delta)
    if delta == 0:
        return "엔코더 신호 없음 -> {} 엔코더 V(3V3)/C(G)/A/B 배선 확인, 모터가 실제로 도는지 확인".format(side)
    if (cmd > 0) != (delta > 0):
        return "방향 반대 -> {} 엔코더 A/B 선을 서로 바꾸거나, 코드에서 부호 반전".format(side)
    return "OK"


def main() -> int:
    ap = argparse.ArgumentParser(description="ESP32 + MDD3A 모터/엔코더 테스트")
    ap.add_argument("--port", default="", help="ESP32 시리얼 포트 (비우면 자동 탐색)")
    ap.add_argument("--speed", type=int, default=120, help="테스트 PWM 0~255 (기본 120)")
    ap.add_argument("--step", type=float, default=2.0, help="동작 한 단계 시간(초)")
    args = ap.parse_args()
    speed = max(0, min(255, args.speed))

    print("== 1. ESP32 찾기 ==")
    port = args.port or find_esp32()
    if not port:
        print("ESP32 를 못 찾았습니다.")
        print("  - ESP32 'UART' 포트에 USB 케이블이 꽂혀 있는지")
        print("  - motor_encoder_test.ino 를 업로드했는지")
        print("  - ls -l /dev/serial/by-id/ 로 장치가 보이는지 확인하세요")
        return 1
    print("  ESP32 포트: {}".format(port))

    try:
        link = Esp32Link(port)
    except serial.SerialException as exc:
        print("포트를 못 열었습니다: {}".format(exc))
        print("  권한 문제면:  sudo usermod -aG dialout $USER  후 로그아웃/로그인")
        return 1

    results = []
    try:
        print("\n== 2. 통신 확인 ==")
        t0 = time.monotonic()
        while link.status is None and time.monotonic() - t0 < 2.0:
            link.poll()
            time.sleep(0.02)
        if link.status is None:
            print("  상태 메시지가 안 옵니다. 보드 리셋(RST) 후 다시 실행하세요.")
            return 1
        print("  OK: ESP32 상태 수신 (체크섬 오류 누적 {}회)".format(link.status.checksum_errors))
        link.reset_encoders()
        time.sleep(0.2)

        print("\n== 3. 모터 + 엔코더 ==  (바퀴가 공중에 떠 있는지 확인!)")
        steps = [
            ("좌측만 정회전", speed, 0),
            ("우측만 정회전", 0, speed),
            ("둘 다 정회전", speed, speed),
            ("둘 다 역회전", -speed, -speed),
            ("정지", 0, 0),
        ]
        for name, l, r in steps:
            results.append(run_step(link, name, l, r, args.step))

        print("\n== 4. 워치독 (명령 끊기면 300ms 안에 정지) ==")
        for _ in range(10):
            link.send_motor(speed, speed)
            time.sleep(0.1)
            link.poll()
        print("  모터 명령을 멈추고 0.6초 기다립니다...")
        time.sleep(0.6)
        link.poll()
        s = link.status
        wd_ok = bool(s and s.watchdog and s.pwm_left == 0 and s.pwm_right == 0)
        print("  {}".format("OK: ESP32 가 스스로 모터를 멈췄습니다" if wd_ok else "실패: 모터가 안 멈춤 -> 펌웨어 확인"))

    except KeyboardInterrupt:
        print("\nCtrl+C: 모터 정지")
        return 130
    finally:
        link.close()

    print("\n== 결과 요약 ==")
    for r in results:
        print("  [{}]".format(r["name"]))
        print("     좌: {}".format(judge(r["left"], r["d_left"], "좌측")))
        print("     우: {}".format(judge(r["right"], r["d_right"], "우측")))
    print("  워치독: {}".format("OK" if wd_ok else "실패"))
    if link.bad_lines:
        print("  체크섬 틀린 수신 줄: {}개 (많으면 케이블/노이즈 확인)".format(link.bad_lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
