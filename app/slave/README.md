# Slave (Implant) — STM32CubeIDE 1.19.0 프로젝트 셋업

체내삽입형 디바이스(Implant) 펌웨어. 이 폴더의 `main.c` 가 펌웨어 진입점이다.

---

## 1. 새 STM32 프로젝트 생성

1. STM32CubeIDE 실행 → **File → New → STM32 Project**
2. **Target Selector**:
   - Series: `STM32U5`
   - Part Number: `STM32U575xx` (Q-series 정확한 부품번호 선택)
3. **Project Setup**:
   - Project Name: `MICS_Slave_Implant`
   - Targeted Language: `C`
   - Targeted Binary Type: `Executable`
   - Targeted Project Type: `STM32Cube`
4. **Firmware Library**: 최신 STM32CubeU5 HAL 패키지 사용
5. **Finish**

---

## 2. .ioc (CubeMX) 페리페럴 설정 — 초저전력 최적화

`MICS_Slave_Implant.ioc` 더블클릭:

### 2.1 클럭 (저전력 우선)
- **System Core → RCC**
  - HSE: **Disable** (절대 사용 금지)
  - HSI16: Disable (필요 시에만)
  - LSE: Crystal Resonator 32.768 kHz (RTC + Stop2 wake-up용)
- **Clock Configuration**
  - SYSCLK Source: **MSI**
  - MSI Range: **4 MHz** (Run 시) — 더 낮은 1 MHz도 가능
  - Voltage Scaling: **Range 4** (최저 전압, Stop2 진입 가능)

### 2.2 SPI
- **SPI1** (CC1101): Run 중에만 활성, 슬립 시 De-Init
  - Pin: PA5(SCK), PA6(MISO), PA7(MOSI)
- **SPI2** (CC2500): WOR 모드에서는 OFF (CC2500 자체가 알아서 RX 함)
  - Pin: PB13(SCK), PB14(MISO), PB15(MOSI)

### 2.3 GPIO (Stop 2 누설 최소화)
| 핀 | 모드 | 설명 |
|----|------|------|
| PA4 | Output Push-Pull, High | CC1101 CS |
| PB12 | Output Push-Pull, High | CC2500 CS |
| PB0 | Input, No Pull (또는 Analog) | CC1101 GDO0 |
| PB1 | **EXTI Mode with Rising trigger, Pull-Down** | CC2500 GDO0 → MCU Wake-up |
| 사용하지 않는 핀 | **Analog** | Stop 2 누설 최소화 |

### 2.4 RTC (주기적 자가점검)
- **Timers → RTC**
  - Activate Clock Source ✔
  - Activate Calendar ✔
  - **WakeUp ✔** (60초 주기)
  - Clock Source: **LSE** (32.768 kHz)

### 2.5 NVIC (Wake-up 소스)
- **EXTI line 1** (PB1, CC2500 GDO0) → Enable, 우선순위 높게
- **RTC Wake-Up Timer** → Enable
- SysTick: Wake-up 후에만 Resume (저전력 모드 진입 시 Suspend)

### 2.6 Power 설정
- **System Core → PWR** (CubeMX에서 직접 설정 어려움 → 코드에서)
- 코드 측에서 `LP_Init()` 호출 시 자동 설정:
  - `HAL_PWREx_EnableUltraLowPowerMode()`
  - `HAL_PWREx_DisableFastWakeUpFromStop()`
  - Voltage Scale 4

설정 완료 → **Project → Generate Code**

---

## 3. 공통 라이브러리 Linked Folder 추가

Master 프로젝트와 동일한 공통 라이브러리를 공유한다.

### 3.1 Source Folder Link

1. Project Explorer → 프로젝트 우클릭 → **New → Folder**
2. **Advanced → Link to alternate location (Linked Folder)** 체크
3. 차례로 추가:
   - `PARENT-2-PROJECT_LOC/cc1101/main` → `cc1101_src`
   - `PARENT-2-PROJECT_LOC/cc2500/main` → `cc2500_src`
   - `PARENT-2-PROJECT_LOC/common`     → `common_src`

### 3.2 Include Path

**Properties → C/C++ Build → Settings → MCU GCC Compiler → Include paths**:

```
../../cc1101/header
../../cc2500/header
../../common
```

---

## 4. 빌드 최적화 설정 (Implant 필수)

**Properties → C/C++ Build → Settings → MCU GCC Compiler**:

| 항목 | 값 | 이유 |
|------|----|----|
| Optimization Level | `-Os` (Optimize for size) | 코드 크기 ↓, Flash 누설 ↓ |
| Debug Level | `Minimal (-g1)` | 릴리즈 빌드 |
| Function sections | ✔ `-ffunction-sections` | 미사용 함수 제거 |
| Data sections | ✔ `-fdata-sections` | 미사용 데이터 제거 |

**MCU GCC Linker → General**:
- ✔ `-Wl,--gc-sections` (Garbage collect unused sections)

**Build Configuration**: `Release` 사용 권장.

---

## 5. main.c 교체

CubeMX 생성 `Core/Src/main.c` 의 `int main(void)` 본문을 이 폴더의 `main.c` 내용으로 대체.

또는 (권장) CubeMX `main()` 의 USER CODE 영역에서 진입점 호출:

```c
/* USER CODE BEGIN Includes */
extern void Slave_AppMain(void);
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
Slave_AppMain();   /* never returns */
/* USER CODE END 2 */
```

이 폴더의 `main.c` 의 `int main(void)` → `void Slave_AppMain(void)` 로 변경,
내부 `HAL_Init()` 제거 (CubeMX 가 이미 호출).

---

## 6. EXTI 콜백 연결

`Core/Src/stm32u5xx_it.c` 의 `EXTI1_IRQHandler()` 가 자동 생성되며,
`HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)` 콜백을 호출한다.
이 폴더 `main.c` 에 정의된 `HAL_GPIO_EXTI_Callback()` 이 약한 심볼을 오버라이드한다.

---

## 7. 빌드 & 플래시

1. **Project → Build All**
2. **Run → Run As → STM32 C/C++ Application**

빌드 산출물: `Release/MICS_Slave_Implant.elf` (또는 `.hex`/`.bin`)

> 💡 **저전력 측정**: 디버거를 분리하고 외부 배터리로 동작시켜야 정확한 ~3.7 uA가 측정됨.
> ST-Link 연결 상태에서는 SWD 라인 누설로 수십 uA 더 측정될 수 있음.
