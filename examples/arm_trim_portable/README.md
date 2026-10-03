# Portable arm trim example

Run `run.ps1` with a C11 host GCC on PATH. The example compiles only the six
generic files (arm_kinematics, arm_collision, arm_trim, each .c and .h) and
the C math library. There are no STM32, Servo, Bluetooth or project-config
dependencies. Its callbacks are a dry run and send no hardware commands.

To adapt an existing application, supply geometry and three linear servo
calibrations in ArmTrimConfig_t, implement send/stop/now, keep ArmTrim_t in
static storage, synchronize from known stable actual joint positions, then
call MoveRelativeX or StartJog and regularly Process. Cancel invalidates the
reference; ReleaseJog finishes normally. Existing transport arbitration and
chassis/input interlocks belong to the caller.

For asynchronous transport, install ArmTrimAsyncIO_t with ArmTrim_SetAsyncIO.
Acceptance is separate from transmit completion. Poll must report actual
STARTED/TC ticks. Cancel must drop queued motion and preserve a complete
active wire frame. The core never treats these timestamps as servo feedback.

DefaultConfig supplies trajectory policy, a disabled collision model and
an enabled range of +/-5 mm. The demo's geometry and calibration are examples;
replace them with the target installation's measurements. Configure a local
collision model explicitly when it is available.
