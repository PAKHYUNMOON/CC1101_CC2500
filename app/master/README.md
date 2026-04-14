# Master (Programmer) — STM32CubeIDE 1.19.0 프로젝트 셋업

체외 장치(Programmer) 펌웨어. 이 폴더의 `main.c` 가 펌웨어 진입점이다.

---

## 1. 새 STM32 프로젝트 생성

1. STM32CubeIDE 실행 → **File → New → STM32 Project**
2. **Target Selector**:
   - Series: `STM32U5`
   - Part Number: `STM32U575xx` (Q-series가 사용하는 정확한 부품번호 선택)
3. **Project Setup**:
   - Project Name: `MICS_Master_Programmer`
   - Targeted Language: `C`
   - Targeted Binary Type: `Executable`
   - Targeted Project Type: `STM32Cube`
4. **Firmware Library**: 최신 STM32CubeU5 HAL 패키지 사용 (예: 1.5.0 이상)
5. **Finish** — `.ioc` 파일과 기본 `Core/`, `Drivers/` 폴더가 생성된다.

---

## 2. .ioc (CubeMX) 페리페럴 설정

`MICS_Master_Programmer.ioc` 더블클릭하여 다음을 설정:

### 2.1 클럭
- **System Core → RCC**
  - HSE: Disable (저전력용 MSI만 사용)
  - LSE: Crystal/Ceramic Resonator (RTC용 32.768 kHz)
- **Clock Configuration**
  - SYSCLK: MSI 4 MHz (저전력 동작 시) ~ 최대 80 MHz
  - PLL은 사용하지 않거나 옵션

### 2.2 SPI (양 라디오)
- **Connectivity → SPI1** (CC1101용)
  - Mode: Full-Duplex Master
  - Baud Rate: ~5 MHz (CC1101 최대 10 MHz)
  - Clock Polarity (CPOL): Low
  - Clock Phase (CPHA): 1 Edge
  - NSS: Disable (수동 GPIO 제어)
  - Pin: PA5(SCK), PA6(MISO), PA7(MOSI)
- **Connectivity → SPI2** (CC2500용)
  - 동일한 설정으로 SPI2 활성화
  - Pin: PB13(SCK), PB14(MISO), PB15(MOSI)

### 2.3 GPIO
| 핀 | 모드 | 설명 |
|----|------|------|
| PA4 | Output Push-Pull, High | CC1101 CS |
| PB12 | Output Push-Pull, High | CC2500 CS |
| PB0 | Input, Pull-Down | CC1101 GDO0 (선택: EXTI) |
| PB1 | EXTI Mode with Rising trigger | CC2500 GDO0 (Wake-up용) |
| PC13 | Output Push-Pull | LED 표시 (선택) |

### 2.4 RTC
- **Timers → RTC**
  - Activate Clock Source ✔
  - Activate Calendar ✔
  - WakeUp ✔ (RTC Wake-Up Timer)
  - Clock Source: LSE

### 2.5 NVIC
- EXTI line 1 (PB1, CC2500 GDO0) 인터럽트 활성화
- RTC Wake-Up Timer 인터럽트 활성화

설정 완료 후 **Project → Generate Code** (또는 Ctrl+S)

---

## 3. 공통 라이브러리 Linked Folder 추가

이 펌웨어는 리포지토리 루트의 `cc1101/`, `cc2500/`, `common/` 폴더를 공유한다.
**복사하지 말고 Link로 참조**한다 (Master/Slave가 같은 코드를 보도록).

### 3.1 Source Folder Link

1. Project Explorer 에서 프로젝트 우클릭 → **New → Folder**
2. **Advanced → Link to alternate location (Linked Folder)** 체크
3. 다음 경로를 차례로 추가:
   - `PARENT-2-PROJECT_LOC/cc1101/main` → 폴더명 `cc1101_src`
   - `PARENT-2-PROJECT_LOC/cc2500/main` → 폴더명 `cc2500_src`
   - `PARENT-2-PROJECT_LOC/common`     → 폴더명 `common_src`

> `PARENT-2-PROJECT_LOC` = 워크스페이스 기준 두 단계 위 (리포지토리 루트).
> 만약 워크스페이스가 리포지토리 루트와 같다면 `PARENT-1-PROJECT_LOC` 사용.

### 3.2 Include Path 추가

프로젝트 우클릭 → **Properties → C/C++ Build → Settings**
→ **MCU GCC Compiler → Include paths** 에 추가:

```
../../cc1101/header
../../cc2500/header
../../common
```

(상대경로 기준은 프로젝트의 `Debug/` 폴더)

---

## 4. main.c 교체

CubeMX가 생성한 `Core/Src/main.c` 의 사용자 코드 영역을 이 폴더의 `main.c` 내용으로 대체한다.

또는 (권장) `Core/Src/main.c` 는 그대로 두고 (HAL_Init, SystemClock_Config, MX_*_Init 호출만 유지),
`/* USER CODE BEGIN 2 */` 영역에 다음을 호출하는 별도 진입점 함수를 추가:

```c
/* Core/Src/main.c 의 USER CODE BEGIN Includes */
#include "master_app.h"   /* 또는 직접 main.c 의 함수 호출 */

/* USER CODE BEGIN 2 */
Master_AppMain();   /* app/master/main.c 의 main() 을 함수로 리네임하여 호출 */
/* USER CODE END 2 */
```

> **간단한 방법**: 이 폴더의 `main.c` 의 `int main(void)` 함수 시그니처를
> `void Master_AppMain(void)` 로 바꾸고, 내부의 `HAL_Init()` 호출은 제거 후
> CubeMX가 생성한 `main()` 끝의 USER CODE 영역에서 `Master_AppMain()` 을 호출한다.

---

## 5. 빌드 & 다운로드

1. **Project → Build All** (Ctrl+B)
2. **Run → Debug As → STM32 C/C++ Application** (또는 ST-Link로 플래시)

빌드 산출물: `Debug/MICS_Master_Programmer.elf`
