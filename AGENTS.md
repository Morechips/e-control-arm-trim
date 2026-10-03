# Repository Guidelines

## Project Overview

Bare-metal STM32F407 firmware for a four-wheel mecanum competition robot: Bluetooth remote control, Emm42 wheel motors, JY61 heading hold, MaxiCam vision/QR, ZL-IS2 servo controller, and route/mission state machines. No RTOS; `Core/Src/main.c` is a single cooperative super-loop whose call order matters (`Car_Control_Process()` runs before and after `Motor_Process()` so safety cancels pending speed before TX dispatch and faults are seen in the same loop).

Code identifiers and comments are English, except the explicitly annotated Chinese action names in servo.c/h. `README.md` and the `*_REPORT.md` files are Chinese and are the authority on pinout, wire-frame layouts, speed tables, and safety behavior — read the relevant section before changing control logic.

## Project Structure & Module Organization

Application firmware lives in `Core/Src/`; public module headers and board configuration live in `Core/Inc/`. Keep related files paired, for example `Core/Src/jy61.c` and `Core/Inc/jy61.h`. Tunable constants and pins go in the matching `Core/Inc/*_config.h` (`car_config.h`, `heading_config.h`, `turn_config.h`, `vision_config.h`, `servo.h`, `start_button_config.h`, `board_input_config.h`, `usart6_config.h`, `laser_config.h`, `arm_config.h`), not inline in logic.

ST CMSIS and HAL dependencies are vendored under `Drivers/`; avoid modifying them unless intentionally updating the vendor package. `cmake/` contains the ARM GCC toolchain support, `openocd/horco-cmsis-dap-stm32f407.cfg` is the probe configuration, `STM32F407_FLASH.ld` is the GCC linker script, and `MDK-ARM/stm32f407_bt_oled.uvprojx` (target `STM32F407_BT_OLED`) is the Keil project.

The build is not the source list. `Core/Src/arm_control.c` and `arm_tuner.c` are legacy modules kept only for host tests; `arm_kinematics_legacy_project.c` supplies their historical project helpers. Production uses the generic `arm_kinematics`, `arm_collision` and `arm_trim` core through `arm_trim_project`, `arm_trim_service` and `arm_trim_input`. Keep the generic core free of project/HAL/transport headers. `Core/Src/x42.c` remains manual-host-test only and excluded from GCC/Keil builds. See `ARM_TRIM_INTEGRATION.md` for current arm behavior and build/flash commands. The car drives motors through `motor_driver`/`emm42_driver`; check `main()` to establish runtime reachability.

`petg_robot_arm_v1/` is a gitignored OpenSCAD + Python mechanical-design bundle with its own `README.md` and its own units (mm) and verification scripts; treat it as a separate sub-project and do not mix it into firmware commits.

Treat `build/`, `build-local/`, `cmake-build-debug/`, and Keil `Objects/` or `Listings/` as generated output. `build/upload-snapshot/` is a stale snapshot copy of the repository, including an obsolete `AGENTS.md`; never edit it or treat anything inside it as current.

## Build, Test, and Development Commands

- `cmake -S . -B build-local -G Ninja -DCMAKE_BUILD_TYPE=Debug` configures an ARM GCC build using `cmake/arm-none-eabi.cmake`.
- `cmake --build build-local --parallel 4` builds `stm32f407_bt_oled.elf` / `.hex` / `.bin` / `.map` and prints memory usage.
- `powershell -ExecutionPolicy Bypass -File tests/uart/run.ps1` tests the shared UART/queue, timestamps, ownership rollback, FIFO/latest, tags, immediate completion and timeout/wrap.
- `powershell -ExecutionPolicy Bypass -File tests/car/run.ps1` runs the optional OLED, start-key, board-input, production/bench laser-input, neutral car-input, remote-heading, car, PD10-standalone, mecanum-mode, USART2-bridge, heading, and right-turn host regressions.
- `powershell -ExecutionPolicy Bypass -File tests/vision/run.ps1` runs the three-digit task-value, detection-layout, and MaxiCam tests.
- `powershell -ExecutionPolicy Bypass -File tests/route/run.ps1` runs the route FSM tests.
- `powershell -ExecutionPolicy Bypass -File tests/zlis2/run.ps1` runs the unified Servo protocol, preset, formatter and queue tests.
- `powershell -ExecutionPolicy Bypass -File tests/servo/run.ps1` runs the Bluetooth-to-Servo dispatch and UART-output regressions with CAR_TEST_INPUTS_ENABLE=0 and 1.
- `powershell -ExecutionPolicy Bypass -File tests/arm/run.ps1` and `tests/arm_kinematics/run.ps1` cover the legacy arm state machine and inverse kinematics.
- `powershell -ExecutionPolicy Bypass -File tests/arm_bt/run.ps1` covers the retired `@ARM` tuner path.
- `tests/arm_trim/run.ps1`, `tests/arm_collision/run.ps1`, `tests/arm_trim_service/run.ps1` and `tests/arm_trim_input/run.ps1` cover generic planning, asynchronous TC, real Servo/UART integration and the independent phone page. `examples/arm_trim_portable/run.ps1` builds with no HAL or project parameters.
- `tests/x42/` has no script; compile it by hand with `gcc -std=c11 -Wall -Wextra -Werror -Itests/x42 -ICore/Inc Core/Src/x42.c tests/x42/test_x42.c -o build-local/test_x42.exe`.

The host scripts require `gcc` on `PATH`; firmware builds require the `arm-none-eabi` toolchain, Ninja, and CMake 3.22+. Host scripts honor `$env:CAR_TEST_BUILD_DIR` (default `build`, supports relative/absolute paths); set it to `build-local` to avoid the existing build directory permissions. Firmware builds above also use `build-local`.

CMake options that change behavior: `USART2_RX_PERIOD_MS` (5 or 2 only), `CAR_MECANUM_TEST_MODE`, `CAR_HEADING_TEST_MODE`, and `CAR_TEST_INPUTS_ENABLE` — CAR_MECANUM_TEST_MODE and CAR_HEADING_TEST_MODE are mutually exclusive and a bad combination is a configure-time `FATAL_ERROR`. Both motion test modes and CAR_PD10_STANDALONE_TEST require CAR_TEST_INPUTS_ENABLE=1. Production uses 0: only PD10 start-key input remains, logs one press, and has no bound mission action; the other physical inputs are bench-only.

## Coding Style & Naming Conventions

Use C11, four-space indentation, and braces on a new line for functions. Match existing naming: `Module_Action()` for public APIs, `snake_case` for local data where established, `UPPER_SNAKE_CASE` for macros, and `PascalCase_t` for typedefs. Header guards use `MODULE_H`. Guard every header's own definitions, keep `static` for file-local helpers, and express compile-time variants as `#ifndef`-overridable macros so host builds can override them.

Firmware compiles with `-Wall -Wextra -Werror` (plus `-Os -g3` and hard-float Cortex-M4 flags); host test builds add `-pedantic`. Everything must build warning-free under both; there is no separate formatter or linter.


UART physical I/O and global HAL callback routing belong only in uart_driver.c (USART2 RX hardware remains in usart2_dma.c). Modules use independent UartTxQueue_t instances and static buffers, not duplicate queue state machines. Event hooks may run in IRQ context: keep them bounded and format logs in foreground. Motor dispatch stays foreground-only and safety cancels pending frames before Motor_Process; servo latest-pending may continue from TC. Preserve each profile's capacity, failure retention, timeout boundary and frame gap.

ServoCode retains 0-8, with REFERENCE/RST/AIM/TH_PRE at 9-12. codes[ServoCode_MAX][4] in servo.c is the single numeric source and uses ServoCommand_t {id,pwm,time_ms}; REFERENCE sends three channels. ServoStatus_t preserves the former status ordinals. Servo is independent of Bluetooth/GPIO and owns protocol formatting plus its shared-queue instance. The former servo_code/servo_pose/servo_remote/zlis2_driver modules and interfaces are removed; historical arm tests use Servo_* APIs.

Servo-owned motion uses an explicit pointer lease, one tracked request at a time and actual STARTED/TC timestamps. Unowned legacy presets keep latest-pending behavior; all unowned commands return BUSY during a lease. Input interlocks run before the service pumps TX. STOP removes queued motion without truncating an active frame; subsequent per-joint stop requests wait for TC/failure. ARM_TRIM_ENABLE defaults off in the input header for selectable integration and is enabled by firmware build configurations. Do not remove this selection or change the original five-field ArmTrimIO_t positional interface.

Bluetooth publishes neutral CarRemoteInput_t snapshots directly to Car_Control_SubmitRemoteInput and dispatches selected servo requests through Bluetooth_DispatchServoActions in the original late-loop position. Functional car code must not read Bluetooth getters/types. Transport error/overflow calls Car_Control_InvalidateRemoteInput immediately, but this ISR-safe API only sets a flag; motor actions stay in foreground safety processing. Legacy arm_tuner Bluetooth getters are the explicit test-only exception.

board_inputs owns GPIO sampling/debounce, including startup hold, local vision/shot edges, standalone PD10, PE4 and PB8. Sample car inputs before the first Car_Control_Process, PE4 in the late servo stage, and PB8/display in Board_Process. Preserve one-shot events despite the double car call, PE4 arbitration, exact input lease and the automatic/manual laser OR policy. Laser_SetManualRequest accepts a request and never reads PB8. PC3 floating GPIO configuration is an output behavior, not input sampling.

Host UART fixtures use tests/uart_hal_defaults.c for selected fallback boundaries; keep test-only flags out of production. Shared host GPIO definitions are in tests/gpio_hal.h.

## Testing Guidelines

Name test files `test_<module>.c` and drive them from a `run.ps1` beside them that compiles against host `gcc` and runs each binary. Place HAL substitutes beside the tests (for example `tests/zlis2/stm32f4xx_hal.h`, `tests/car/stm32f4xx_hal.h`) and keep production code free of host-only branches when a stub can provide the boundary. Shared fixtures live under `tests/`: `tests/phone20_packet.h` (wire-frame builders) and `tests/car/mock_maxicam.c` / `mock_jy61_transport.c` are reused by several suites, which is why `tests/vision`, `tests/route`, and `tests/car` all put `tests/car` on the include path.

Run the affected suite plus a full firmware build before submitting. No coverage threshold is configured; add regression cases for state transitions, parsing, timeouts, and safety behavior. Passing host tests proves logic only — they say nothing about motor direction, mechanical stopping, or sensor polarity.

## Safety Constraints

`CAR_BOOT_AUTO_ENABLE=1` means the default firmware energizes all four wheels at power-up and only accepts motion after a valid centred control frame. Never flash, power a drivetrain, or enable motors without confirming the target board, wiring, and safe test conditions, and never raise a low-speed motion constant to make a bench test "work" without recording it — the README documents which speeds are deliberately unverified on hardware.

## Commit & Pull Request Guidelines

Use short, imperative, scoped subjects such as `car: reject stale control packets`, `mission: complete visual tasks and QR scan route`. Keep commits focused and exclude generated binaries (`build/`, `cmake-build-*`, Keil outputs) and the gitignored `petg_robot_arm_v1/` bundle. Pull requests should describe behavior and hardware impact, list the exact commands run, link relevant issues, and include serial logs or bench-test evidence for device-facing changes.
