# motor_test

순찰 로봇(라즈베리파이 4 + ESP32-S3 + MDD3A)의 배선이 제대로 됐는지 확인하는 코드입니다.

| 파일 | 하는 일 |
| --- | --- |
| `codes/check_connections.sh` | 라즈베리파이에서 장치(USB 시리얼, GPS UART, I2C, 카메라, ROS 2)가 보이는지 한 번에 확인 |
| `setuptest.py` | GY-85, GPS, LD2410C 레이더, 카메라 값 읽기 ([s1_patrol_robot](https://github.com/ornithopter-hb/s1_patrol_robot) 에서 가져옴) |
| `esp32/motor_encoder_test/motor_encoder_test.ino` | ESP32 펌웨어: MDD3A 모터 2개 구동, 엔코더 읽기, 체크섬, 300ms 워치독 |
| `codes/motor_test.py` | 라즈베리파이에서 ESP32 로 모터 명령을 보내고 엔코더 값·방향·워치독 확인 |

사용법은 [docs/연결확인_가이드.md](docs/연결확인_가이드.md) 를 순서대로 따라 하면 됩니다.

```bash
git clone https://github.com/joyrtree/motor_test.git
cd motor_test
bash codes/check_connections.sh
```
