# TODO

- 추가 
  - [x] Micro SD Card 기능 추가
  - [x] 기압 센서 추가
  - [x] PWM Channel 추가(기존 4개/목표 6개)
  - [x] I2C & CAN 인터페이스 추가(고도 측정용 LiDAR, 피토관 센서 등)
  - [x] Pixhawk 6 PM 인터페이스 추가
- 개량
  - [x] 자이로/가속도/지자계 센서 개량 및 업그레이드
    - [x] 자이로1: ICM-42688-P (SPI1)
    - [x] 자이로2 : BMI323 (SPI3)
    - [x] 지자계 : LIS2MDLTR (SPI3)
  - [x] iBus/SRXL2 수신기 연동
  - [x] 센서 연결 커낵터를 Pixhawk 6C Mini에 맞게 수정 (GPS/Tele./I2C etc..)
	- GPS1 일부 배선 미결선
  - [x] USB C type 커낵터 수정
- TEST
  - [ ] PM02 Analog data read
  - [ ] SRXL2 UART half-duplex data read&write

# Update Log

## 2025.01.30:SRXL2 connect with AUX1/PCB Routing
- AUX1

## 2025.01.03:Big Update:IMU/RC/Conn./MCU
- PWM Channel 추가(기존 4개/목표 6개)
- I2C & CAN 인터페이스 추가(고도 측정용 LiDAR, 피토관 센서 등)
- 자이로/가속도/지자계 센서 개량 및 업그레이드
- iBus/SRXL2 수신기 연동
- Spektrum BAT 커넥터 추가

## 2025.01.03v2:PM(Power Module) added
- Power Module 인터페이스 추가

## 2025.01.06:Baro/SD Card update
- GPS1 SW/LED/BUZZER 결선
- SD Card update
- 소자 Footprint update

