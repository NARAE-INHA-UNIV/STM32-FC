**본 문서는 STM32 firmware만을 다루고 있습니다.**

> [[STM32-FC-docs]](https://github.com/NARAE-INHA-UNIV/STM32-FC-docs) How to use, Specification <br>
> [[STM32-FC-hw]](https://github.com/NARAE-INHA-UNIV/STM32-FC-hw) KiCAD, Schematic, PCB Artwork <br>
> [[MiniLink_cli]](https://github.com/NARAE-INHA-UNIV/MiniLink_cli) Data analysis like msg, log, param <br>

-----

# STM32 FC


![pcb img](./STM32-FC.jpg)


인하대학교 모형항공기 동아리 나래에서 2024년 동계 UAV 활동으로 진행한 프로젝트임. <br>
회로와 펌웨어를 개발하되, 기성 pixhawk 모듈과 호환성을 가지기 위해 pixhawk 6C mini의 인터페이스와 흡사하게 제작하는 것을 목표로함. <br>

- 활동명 : [UAV] FC 제작 활동
- 일시 : 2024 동계 방학 - 진행중
- 팀명 :  항공전자 (구 FC)
- 팀장 : 정영우(2025), 이현제(2024)


## 폴더 구조

`Plane` 폴더가 STM32Cube IDE 프로젝트이다. 주로 수정할 코드는 `Core` 디렉토리 내에 위치한다.

`Plane` -> `Core` -> `Src/Inc`

- Src : Source code (.c)
- Inc : header (.h)
  - {Module Name}.h
  - driver.h

{Module Name} 앞의 접두사의 의미 :
- `FC` : 보드 내부 HW 관리 (UART, RC, Servo etc.)
- `AP` : 고수준의 제어기나 자율 비행 (Failsafe, RTH etc.)
- `SEN` : 외부 센서

헤더 파일 중 `driver.h`는 외부 모듈에서 접근할 떄 사용한다. <br>
예를 들어 `main.c`가 `RadioContol` 모듈에 접근할 때 `FC_RC/driver.h`에 접근해야 한다. `RadioContol.h`는 해당 모듈 내부에서 사용하는 코드로 외부에서 직접 접근하지 않도록 한다.

```bash
├─AP_Failsafe       # Failsafe 관련 코드
├─FC_AHRS           # IMU 센서, 기압 센서
├─FC_Basic          # Buzzer나 LED 등
├─FC_RC             # 라디오 조종기 (스펙트럼, PPM, ELRS)
├─FC_Servo          # 서보 모터, ESC
├─FC_Serial         # Serial 기능
│  └─MiniLink       # 외부 장치와 통신하는 프로토콜
│      └─Param      # 파라미터 관리
└─test              # 테스트 중인 코드
```


## 기여 (개발)

1. 프로젝트 fork
2. fork된 개인 레파지토리 clone
3. 코드 작성
4. 개인 레파지토리에 커밋 & 푸시
5. Pull-request
