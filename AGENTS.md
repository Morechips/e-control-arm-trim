# Repository Guidelines

## Project Structure & Module Organization

Application firmware lives in `Core/Src/`; public module headers and board configuration live in `Core/Inc/`. Keep related files paired, for example `Core/Src/jy61.c` and `Core/Inc/jy61.h`. ST CMSIS and HAL dependencies are vendored under `Drivers/`; avoid modifying them unless intentionally updating the vendor package. `cmake/` contains the ARM GCC toolchain support, `openocd/` contains probe configuration, and `MDK-ARM/` is the Keil project. Host-side tests are grouped by feature in `tests/car`, `tests/zlis2`, and `tests/x42`. Treat `build/` and Keil `Objects/` or `Listings/` as generated output.

## Build, Test, and Development Commands

- `cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug` configures an ARM GCC build using `cmake/arm-none-eabi.cmake`.
- `cmake --build build --parallel 4` builds the `.elf`, `.hex`, and `.bin` firmware artifacts and reports memory usage.
- `powershell -ExecutionPolicy Bypass -File tests/car/run.ps1` compiles and runs the car, mecanum, heading, and UART bridge host regressions.
- `powershell -ExecutionPolicy Bypass -File tests/zlis2/run.ps1` runs the ZL-IS2 driver tests.

The host scripts require `gcc` on `PATH`; firmware builds require the `arm-none-eabi` toolchain and Ninja. Keil users can rebuild `MDK-ARM/stm32f407_bt_oled.uvprojx`.

## Coding Style & Naming Conventions

Use C11, four-space indentation, and braces on a new line for functions. Match existing naming: `Module_Action()` for public APIs, `snake_case` for local data where established, `UPPER_SNAKE_CASE` for macros, and `PascalCase_t` for typedefs. Keep hardware-facing constants in the relevant `*_config.h`. Compile cleanly with `-Wall -Wextra -Werror`; no separate formatter is configured.

## Testing Guidelines

Name test files `test_<module>.c`. Place HAL substitutes beside the tests (for example, `tests/zlis2/stm32f4xx_hal.h`) and keep production code free of host-only branches when a stub can provide the boundary. Run the affected suite plus a full firmware build before submitting. No coverage threshold is configured; add regression cases for state transitions, parsing, timeouts, and safety behavior.

## Commit & Pull Request Guidelines

Git history is not exposed in this checkout. Use short, imperative, scoped subjects such as `car: reject stale control packets`. Keep commits focused and exclude generated binaries. Pull requests should describe behavior and hardware impact, list commands run, link relevant issues, and include serial logs or bench-test evidence for device-facing changes. Never flash or enable motors without confirming the target board, wiring, and safe test conditions.
