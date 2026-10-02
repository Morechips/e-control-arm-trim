#ifndef ARM_TRIM_BENCH_H
#define ARM_TRIM_BENCH_H
#include <stdbool.h>
#include <stdint.h>
/* Test-copy-only helper, separate from the portable trim core. PREP moves only
 * 000..002; READY does the same then auto-synchronizes. Both preserve 003.
 * CLOSE/OPEN address only 003 and preserve the planar reference. No startup action. */
bool ArmTrimBench_HandleLine(const char *line, uint32_t arrival_tick, bool legacy_session);
bool ArmTrimBench_IsActive(void);
void ArmTrimBench_Process(void);
void ArmTrimBench_Cancel(void);
#endif
