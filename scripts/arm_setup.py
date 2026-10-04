"""Installation wizard and read-only bus-servo pose recorder (Python 3.9+).

Serial access is confined to arm_serial_capture. No movement, release, reset,
flash or build command is sent by this program. All lengths are millimeters;
calibration input is degrees, with relative joint angles for 001 and 002.
"""
import argparse
import copy
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sys

from arm_serial_capture import CaptureError, capture_positions, list_ports

ROOT = Path(__file__).resolve().parent.parent
PROFILES = ("BALL", "HOSTAGE", "BUCKET")
DEFAULT_CONFIG = ROOT / "configs" / "arm_installation.json"
NAME_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,47}\Z")


class ConfigError(ValueError):
    pass


def current_example():
    """Explicit example of the v4.6 installation, not an auto-measured model."""
    return {
        "schema_version": 1, "name": "current_v4_6_example",
        "geometry": {"link_1_mm": 104.85, "link_2_mm": 84.75,
                     "tool_x_mm": 121.1538, "tool_z_mm": 52.4,
                     "tool_axis_offset_deg": 0.0},
        "joints": [
            {"id": 0, "min_pwm": 915, "max_pwm": 1800,
             "calibration": [{"pwm": 989, "angle_deg": 0.0}, {"pwm": 1309, "angle_deg": 45.0}]},
            {"id": 1, "min_pwm": 821, "max_pwm": 2500,
             "calibration": [{"pwm": 1687, "angle_deg": 0.0}, {"pwm": 2344, "angle_deg": -90.0}]},
            {"id": 2, "min_pwm": 500, "max_pwm": 1874,
             "calibration": [{"pwm": 1232, "angle_deg": 0.0}, {"pwm": 571, "angle_deg": -90.0}]}],
        "references": {
            "BALL": {"positions": [1356, 1850, 698], "confirmed": False, "note": "待实机验收"},
            "HOSTAGE": {"positions": [1684, 2136, 785], "confirmed": False, "note": "旧安装值，待重录"},
            "BUCKET": {"positions": [1566, 1896, 673], "confirmed": False, "note": "待实机验收"}},
        "gripper": {"id": 3, "min_pwm": 500, "max_pwm": 2500,
                    "close_pwm": 500, "open_pwm": 1800},
        "collision": {"enabled": True, "box_min_mm": [-70.0, -50.0, -31.2],
                      "box_max_mm": [-30.0, 50.0, 38.8], "radii_mm": [15.0, 20.0, 60.0],
                      "envelope_estimated": True, "clearance_mm": 5.0,
                      "sweep_resolution_mm": 0.5},
        "trim": {"search_mm": 75.0, "min_offset_mm": -75.0, "max_offset_mm": 75.0,
                 "max_segment_mm": 2.0, "speed_mm_s": 10.0, "acceleration_mm_s2": 20.0,
                 "update_period_ms": 50, "settle_ms": 300},
        "timing": {"reference_move_ms": 2000, "reference_guard_ms": 300,
                   "grip_move_ms": 1500, "grip_guard_ms": 300},
        "action_groups": []}


def new_installation():
    result = current_example()
    result["name"] = "new_installation"
    result["geometry"] = {key: None for key in result["geometry"]}
    for joint in result["joints"]:
        joint["min_pwm"] = joint["max_pwm"] = None
        joint["calibration"] = [{"pwm": None, "angle_deg": None}, {"pwm": None, "angle_deg": None}]
    for profile in result["references"].values():
        profile.update(positions=None, confirmed=False, note="未录入")
    result["gripper"].update(min_pwm=None, max_pwm=None, close_pwm=None, open_pwm=None)
    result["collision"]["enabled"] = False
    for key in ("box_min_mm", "box_max_mm", "radii_mm"):
        result["collision"][key] = [None, None, None]
    result["collision"]["clearance_mm"] = None
    result["trim"].update(min_offset_mm=-5.0, max_offset_mm=5.0)
    return result


def load_config(path):
    try:
        result = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as error:
        raise ConfigError(f"不能读取配置 {path}: {error}") from error
    if not isinstance(result, dict) or result.get("schema_version") != 1:
        raise ConfigError("仅支持 schema_version=1 的配置")
    return result


def save_config(path, config, replace=True):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if not replace and path.exists():
        raise ConfigError(f"文件已存在，不覆盖: {path}")
    if path.exists():
        # Keep the last state recoverable even after an interrupted wizard.
        backup = path.with_name(path.name + ".bak")
        shutil.copy2(path, backup)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def number(value, label, low=None, high=None, integer=False):
    valid_type = type(value) is int if integer else type(value) in (int, float)
    if not valid_type:
        raise ConfigError(f"{label}: 需要{'整数' if integer else '有限数值'}，不能留空")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite or abs(value) > 3.4028234663852886e38 or (value != 0 and abs(value) < 1.1754943508222875e-38):
        raise ConfigError(f"{label}: 数值不能安全表示为固件C float")
    if (low is not None and value < low) or (high is not None and value > high):
        raise ConfigError(f"{label}: {value} 不在允许范围 {low}..{high}")
    return value


def pwm(value, label, limits=(500, 2500)):
    return number(value, label, *limits, integer=True)


def vector(values, label):
    if not isinstance(values, list) or len(values) != 3:
        raise ConfigError(f"{label}: 需要三个数值")
    return [number(value, f"{label}[{index}]") for index, value in enumerate(values)]


def joint_limits(config):
    joints = config.get("joints")
    if not isinstance(joints, list) or len(joints) != 3:
        raise ConfigError("当前模块只支持 000/001/002 三个平面关节")
    limits = {}
    for index, joint in enumerate(joints):
        if type(joint.get("id")) is not int or joint["id"] != index:
            raise ConfigError("joints 必须依序为 id=0,1,2；当前固件不支持重映射")
        low = pwm(joint.get("min_pwm"), f"{index:03} 最小P")
        high = pwm(joint.get("max_pwm"), f"{index:03} 最大P")
        if low >= high:
            raise ConfigError(f"{index:03} 最小P必须小于最大P")
        limits[index] = (low, high)
    return limits


def validate(config):
    """Validate firmware-supported dimensions; no physical-safety claim."""
    warnings = []
    try:
        if config.get("schema_version") != 1:
            raise ConfigError("schema_version 必须为1")
        if not isinstance(config.get("name"), str) or not NAME_PATTERN.fullmatch(config["name"]):
            raise ConfigError("name 使用字母开头的英文、数字、下划线，最多48字符")
        g = config["geometry"]
        number(g["link_1_mm"], "L1", 0.001)
        number(g["link_2_mm"], "L2", 0.001)
        for key in ("tool_x_mm", "tool_z_mm", "tool_axis_offset_deg"):
            number(g[key], key)
        limits = joint_limits(config)
        for index, joint in enumerate(config["joints"]):
            pairs = joint["calibration"]
            if len(pairs) != 2:
                raise ConfigError(f"{index:03} 必须有两个标定点")
            for point in pairs:
                pwm(point["pwm"], f"{index:03} 标定P", limits[index])
                number(point["angle_deg"], f"{index:03} 标定角度")
            if pairs[0]["pwm"] == pairs[1]["pwm"] or abs(pairs[0]["angle_deg"] - pairs[1]["angle_deg"]) < 0.001:
                raise ConfigError(f"{index:03} 两个标定P、两个标定角均必须不同")
        if set(config["references"]) != set(PROFILES):
            raise ConfigError("references 必须包含 BALL/HOSTAGE/BUCKET 三组")
        for name in PROFILES:
            ref = config["references"][name]
            values = ref["positions"]
            if not isinstance(values, list) or len(values) != 3:
                raise ConfigError(f"{name}: 必须录000/001/002，禁止夹爪003")
            for index, value in enumerate(values):
                pwm(value, f"{name} {index:03}", limits[index])
            if type(ref.get("confirmed")) is not bool:
                raise ConfigError(f"{name}.confirmed 必须为布尔值")
            if not ref["confirmed"]:
                warnings.append(f"{name} 参考待实机确认: {ref.get('note', '')}")
        grip = config["gripper"]
        if grip["id"] != 3:
            raise ConfigError("当前夹爪通道固定为003")
        grip_limits = (pwm(grip["min_pwm"], "夹爪最小P"), pwm(grip["max_pwm"], "夹爪最大P"))
        if grip_limits[0] >= grip_limits[1]:
            raise ConfigError("夹爪最小P必须小于最大P")
        for key in ("close_pwm", "open_pwm"):
            pwm(grip[key], key, grip_limits)
        if grip["close_pwm"] == grip["open_pwm"]:
            raise ConfigError("夹紧/松开不能使用相同P")
        limits[3] = grip_limits
        collision = config["collision"]
        if type(collision["enabled"]) is not bool or type(collision["envelope_estimated"]) is not bool:
            raise ConfigError("碰撞开关和估计标记必须为布尔值")
        if collision["enabled"]:
            mins, maxs = vector(collision["box_min_mm"], "箱体最小坐标"), vector(collision["box_max_mm"], "箱体最大坐标")
            if any(a >= b for a, b in zip(mins, maxs)):
                raise ConfigError("箱体各轴最小值必须小于最大值")
            if any(value < 0 for value in vector(collision["radii_mm"], "包络半径")):
                raise ConfigError("包络半径不能为负数")
            number(collision["clearance_mm"], "额外间隙", 0)
        number(collision["sweep_resolution_mm"], "扫描分辨率", 0.1, 2.0)
        if not collision["enabled"]:
            warnings.append("未启用碰撞模型，软件不检查该箱体")
        elif collision["envelope_estimated"]:
            warnings.append("碰撞包络含估计值，尚不能视为实测安全边界")
        trim = config["trim"]
        search = number(trim["search_mm"], "搜索范围", 1, 75)
        number(trim["min_offset_mm"], "微调负边界", -search, 0)
        number(trim["max_offset_mm"], "微调正边界", 0, search)
        number(trim["max_segment_mm"], "最大分段", 0.1, 2)
        number(trim["speed_mm_s"], "速度", 0.001, 10)
        number(trim["acceleration_mm_s2"], "加速度", 0.001, 20)
        # Dispatch lateness remains 20 ms in the generic defaults.
        number(trim["update_period_ms"], "周期", 21, 100, integer=True)
        number(trim["settle_ms"], "微调稳定等待", 0, 60000, integer=True)
        for key, value in config["timing"].items():
            if key not in ("reference_move_ms", "reference_guard_ms", "grip_move_ms", "grip_guard_ms"):
                raise ConfigError(f"未知 timing 配置 {key}")
            number(value, key, 1 if "move" in key else 0, 9999 if "move" in key else 60000, integer=True)
        if len(config["timing"]) != 4:
            raise ConfigError("timing 缺少动作或稳定等待时间")
        names = set()
        for group in config["action_groups"]:
            name = group["name"]
            if not NAME_PATTERN.fullmatch(name) or name in names:
                raise ConfigError("动作组名称须为唯一的英文/数字/下划线，字母开头")
            names.add(name)
            if not group["points"]:
                raise ConfigError(f"{name} 没有动作点")
            for index, point in enumerate(group["points"]):
                channels = point["positions"]
                expected = {"0", "1", "2"} if point["kind"] == "planar" else {"3"} if point["kind"] == "gripper" else set()
                if not expected or not isinstance(channels, dict) or set(channels) != expected:
                    raise ConfigError(f"{name}[{index}] 平面步骤仅0/1/2；夹爪步骤仅3")
                for channel, value in channels.items():
                    pwm(value, f"{name}[{index}] 通道{channel}", limits[int(channel)])
                number(point["time_ms"], "动作点时间", 1, 9999, integer=True)
                number(point["guard_ms"], "动作点等待", 0, 60000, integer=True)
        warnings.append("校验仅检查数据/协议范围；固定动作路径及实机碰撞仍须验证")
    except (KeyError, TypeError, AttributeError) as error:
        raise ConfigError(f"配置缺项或类型错误: {error}") from error
    return warnings


def macro_file(guard, macros):
    lines = [f"#ifndef {guard}", f"#define {guard}", "",
             "/* Generated by scripts/arm_setup.py. Review before building. */"]
    for name, value in macros.items():
        lines += [f"#ifndef {name}", f"#define {name} {value}", "#endif"]
    return "\n".join(lines + ["", "#endif", ""])


def c_float(value):
    text = format(float(value), ".9g")
    if "." not in text and "e" not in text:
        text += ".0"
    return f"({text}f)" if value < 0 else text + "f"


def c_uint(value):
    return f"{value}U"


def headers(config):
    validate(config)
    g, trim = config["geometry"], config["trim"]
    macros = {
        "ARM_TRIM_PROJECT_LINK_1_MM": c_float(g["link_1_mm"]),
        "ARM_TRIM_PROJECT_LINK_2_MM": c_float(g["link_2_mm"]),
        "ARM_TRIM_PROJECT_TOOL_X_MM": c_float(g["tool_x_mm"]),
        "ARM_TRIM_PROJECT_TOOL_Z_MM": c_float(g["tool_z_mm"]),
        "ARM_TRIM_PROJECT_TOOL_AXIS_OFFSET_RAD": c_float(math.radians(g["tool_axis_offset_deg"]))}
    for joint in config["joints"]:
        index = joint["id"]
        for suffix, key in (("MIN", "min_pwm"), ("MAX", "max_pwm")):
            macros[f"ARM_TRIM_PROJECT_P{index}_{suffix}"] = c_uint(joint[key])
        for suffix, pair in zip(("A", "B"), joint["calibration"]):
            macros[f"ARM_TRIM_PROJECT_P{index}_{suffix}"] = c_uint(pair["pwm"])
            macros[f"ARM_TRIM_PROJECT_Q{index}_{suffix}_DEG"] = c_float(pair["angle_deg"])
    for suffix, key in (("SEARCH_MM", "search_mm"), ("MIN_MM", "min_offset_mm"),
                        ("MAX_MM", "max_offset_mm"), ("MAX_SEGMENT_MM", "max_segment_mm"),
                        ("SPEED_MM_S", "speed_mm_s"), ("ACCELERATION_MM_S2", "acceleration_mm_s2")):
        macros["ARM_TRIM_PROJECT_" + suffix] = c_float(trim[key])
    macros["ARM_TRIM_PROJECT_UPDATE_PERIOD_MS"] = c_uint(trim["update_period_ms"])
    macros["ARM_TRIM_PROJECT_SETTLE_MS"] = c_uint(trim["settle_ms"])
    for name in PROFILES:
        for index, value in enumerate(config["references"][name]["positions"]):
            macros[f"ARM_TRIM_{name}_P{index}"] = c_uint(value)
    grip = config["gripper"]
    for macro, key in (("ARM_TRIM_GRIPPER_MIN_P", "min_pwm"), ("ARM_TRIM_GRIPPER_MAX_P", "max_pwm"),
                       ("ARM_TRIM_BENCH_CLOSE_P", "close_pwm"), ("ARM_TRIM_BENCH_OPEN_P", "open_pwm")):
        macros[macro] = c_uint(grip[key])
    collision = config["collision"]
    if not collision["enabled"]:
        # These generated constants are inactive placeholders, not measurements.
        collision = {**collision, "box_min_mm": [-1.0] * 3,
                     "box_max_mm": [1.0] * 3, "radii_mm": [0.0] * 3, "clearance_mm": 0.0}
    cm = {"ARM_COLLISION_PROJECT_ENABLED": str(int(collision["enabled"])),
          "ARM_COLLISION_ENVELOPE_ESTIMATED": str(int(collision["envelope_estimated"]))}
    for index, axis in enumerate("XYZ"):
        cm[f"ARM_REAR_BOX_{axis}_MIN_MM"] = c_float(collision["box_min_mm"][index])
        cm[f"ARM_REAR_BOX_{axis}_MAX_MM"] = c_float(collision["box_max_mm"][index])
    for name, value in zip(("LINK1", "LINK2", "TOOL"), collision["radii_mm"]):
        cm[f"ARM_COLLISION_{name}_RADIUS_MM"] = c_float(value)
    cm["ARM_COLLISION_CLEARANCE_MM"] = c_float(collision["clearance_mm"])
    cm["ARM_COLLISION_SWEEP_RESOLUTION_MM"] = c_float(collision["sweep_resolution_mm"])
    timing = config["timing"]
    im = {"ARM_TRIM_INPUT_LEASE_MS": "500U",
          "ARM_TRIM_PROFILE_MOVE_MS": c_uint(timing["reference_move_ms"]),
          "ARM_TRIM_PROFILE_GUARD_MS": c_uint(timing["reference_guard_ms"]),
          "ARM_TRIM_GRIP_MOVE_MS": c_uint(timing["grip_move_ms"]),
          "ARM_TRIM_GRIP_GUARD_MS": c_uint(timing["grip_guard_ms"])}
    return {"arm_trim_project_config.h": macro_file("ARM_TRIM_PROJECT_CONFIG_H", macros),
            "arm_collision_config.h": macro_file("ARM_COLLISION_CONFIG_H", cm),
            "arm_trim_input_config.h": macro_file("ARM_TRIM_INPUT_CONFIG_H", im)}


def action_header(config):
    lines = ["#ifndef ARM_RECORDED_ACTIONS_H", "#define ARM_RECORDED_ACTIONS_H", "",
             "#include <stddef.h>", "#include <stdint.h>", "",
             "/* Data only. Dispatch one step, wait for TC + T + guard, then next.",
             " * Never enqueue every step through the latest-pending Servo API. */",
             "typedef struct { uint16_t pwm[4], time_ms, guard_ms; uint8_t mask; } ArmRecordedStep_t;",
             "typedef struct { const char *name; const ArmRecordedStep_t *steps; size_t count; } ArmRecordedGroup_t;", ""]
    for index, group in enumerate(config["action_groups"]):
        lines.append(f"static const ArmRecordedStep_t arm_recorded_steps_{index}[] = {{")
        for point in group["points"]:
            values = [point["positions"].get(str(channel), 0) for channel in range(4)]
            mask = sum(1 << int(channel) for channel in point["positions"])
            lines.append("    {{%s}, %dU, %dU, %dU}," % (", ".join(f"{value}U" for value in values), point["time_ms"], point["guard_ms"], mask))
        lines.append("};")
    if config["action_groups"]:
        lines += ["", "static const ArmRecordedGroup_t arm_recorded_groups[] = {"]
        for index, group in enumerate(config["action_groups"]):
            lines.append(f'    {{"{group["name"]}", arm_recorded_steps_{index}, {len(group["points"])}U}},')
        lines += ["};", "#define ARM_RECORDED_GROUP_COUNT (sizeof(arm_recorded_groups) / sizeof(arm_recorded_groups[0]))"]
    else:
        lines.append("#define ARM_RECORDED_GROUP_COUNT 0U")
    return "\n".join(lines + ["", "#endif", ""])


def export_config(config, output):
    warnings = validate(config)
    output = Path(output)
    is_inc = output.name.lower() == "inc" and output.parent.name.lower() == "core"
    if (output / "Core").exists() or is_inc:
        raise ConfigError("export 只能写预览目录；替换工程配置请用 apply")
    if any((output / name).exists() for name in headers(config)):
        try:
            prior = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            prior = {}
        if prior.get("generator") != "arm_setup_v1":
            raise ConfigError("目标已有非本工具生成的配置头，请选择空预览目录")
    output.mkdir(parents=True, exist_ok=True)
    files = headers(config)
    files["arm_recorded_actions.h"] = action_header(config)
    files["installation.json"] = json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    for name, content in files.items():
        (output / name).write_text(content, encoding="utf-8")
    with (output / "action_steps.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["group", "step", "kind", "P000", "P001", "P002", "P003", "time_ms", "guard_ms"])
        for group in config["action_groups"]:
            for index, point in enumerate(group["points"]):
                writer.writerow([group["name"], index + 1, point["kind"],
                                 *(point["positions"].get(str(channel), "") for channel in range(4)),
                                 point["time_ms"], point["guard_ms"]])
    commands = ["# Preview only. These lines are NOT sent to any port."]
    for group in config["action_groups"]:
        commands.append(f"# GROUP {group['name']}")
        for point in group["points"]:
            commands.append("{" + "".join(f"#{int(channel):03}P{value:04}T{point['time_ms']:04}!" for channel, value in sorted(point["positions"].items())) + "}")
            commands.append(f"# After UART completion wait at least T={point['time_ms']} + guard={point['guard_ms']} ms")
    (output / "action_commands.txt").write_text("\n".join(commands) + "\n", encoding="utf-8")
    digest = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in output.iterdir() if path.is_file() and path.name != "manifest.json"}
    (output / "manifest.json").write_text(json.dumps({"schema_version": 1, "generator": "arm_setup_v1", "files_sha256": digest}, indent=2) + "\n", encoding="utf-8")
    print(f"已导出到 {output}")
    for warning in warnings:
        print("提醒: " + warning)


def apply_config(config, project):
    files = headers(config)  # Validate everything before any project mutation.
    project = Path(project).resolve()
    inc = project / "Core" / "Inc"
    for name in files:
        if not (inc / name).is_file():
            raise ConfigError(f"不是可接入的工程，缺少 {inc / name}")
    adapter = (project / "Core" / "Src" / "arm_trim_project.c").read_text(encoding="utf-8-sig")
    service_header = (inc / "arm_trim_service.h").read_text(encoding="utf-8-sig")
    input_source = (project / "Core" / "Src" / "arm_trim_input.c").read_text(encoding="utf-8-sig")
    if "ARM_TRIM_PROJECT_Q0_A_DEG" not in adapter or "grip_min_pwm" not in service_header or "ARM_TRIM_GRIPPER_MIN_P" not in input_source:
        raise ConfigError("此工程尚未接入可配置标定角/夹爪限位，不能直接应用")
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup = project / "build-local" / "arm_setup" / ("backup-" + stamp)
    backup.mkdir(parents=True)
    for name in files:
        shutil.copy2(inc / name, backup / name)
    # Write all temporary files before replacing the three reviewed headers.
    try:
        for name, content in files.items():
            (inc / (name + ".setup.tmp")).write_text(content, encoding="utf-8")
        for name in files:
            (inc / (name + ".setup.tmp")).replace(inc / name)
    except (OSError, KeyboardInterrupt):
        for name in files:
            shutil.copy2(backup / name, inc / name)
        raise
    finally:
        for name in files:
            temporary = inc / (name + ".setup.tmp")
            if temporary.exists():
                temporary.unlink()
    print(f"已应用三份配置；原文件备份: {backup}")
    print("尚未构建或烧录；自定义多步动作组未写入固件执行器。")


def ask(label, default=None, kind=float):
    while True:
        raw = input(f"{label}" + (f" [{default}]" if default is not None else "") + ": ").strip()
        if not raw and default is not None:
            return default
        try:
            value = kind(raw)
            if kind in (int, float) and not math.isfinite(value):
                raise ValueError()
            return value
        except ValueError:
            print("请输入有效数值。")


def yes(label, default=False):
    raw = input(label + (" [Y/n]: " if default else " [y/N]: ")).strip().lower()
    return default if not raw else raw in ("y", "yes", "是")


def connect_capture(port, ids, baud, timeout):
    if not port:
        available = list_ports()
        for item in available:
            print(f"{item['device']}: {item['description']}")
        port = input("控制板端口，如 COM13: ").strip()
    print("请先把机械臂调到目标姿态并等待稳定，关闭占用该COM口的软件。")
    print("USB/TTL直连舵机板；断开STM32到舵机板的串口，确认供电、交叉TX/RX及共地。")
    if not yes("确认以上条件，仅查询当前位置，不发送运动命令？"):
        raise ConfigError("本次未读取串口")
    values = capture_positions(port, ids, baud=baud, timeout_s=timeout)
    print("回读: " + " ".join(f"#{channel:03}P{value:04}!" for channel, value in sorted(values.items())))
    return values, {"source": "bus_PRAD", "port": port, "baud": baud,
                    "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    "samples": 2, "tolerance_pwm": 2}


def capture_point(config, values, group_name, kind, time_ms, guard_ms, source, profile=None):
    """Apply only a complete valid snapshot; never save partial capture."""
    result = copy.deepcopy(config)
    expected = {0, 1, 2} if kind == "planar" else {3}
    if set(values) != expected:
        raise ConfigError("采集通道不完整或混入夹爪")
    limits = joint_limits(result) if kind == "planar" else {3: (pwm(result["gripper"]["min_pwm"], "夹爪最小P"), pwm(result["gripper"]["max_pwm"], "夹爪最大P"))}
    for channel, value in values.items():
        pwm(value, f"通道{channel}", limits[channel])
    number(time_ms, "动作时间", 1, 9999, integer=True)
    number(guard_ms, "稳定等待", 0, 60000, integer=True)
    if not NAME_PATTERN.fullmatch(group_name):
        raise ConfigError("动作组名称须字母开头，仅英文、数字、下划线，最多48字符")
    if profile is not None and (kind != "planar" or profile not in PROFILES):
        raise ConfigError("只有平面姿态能设置为BALL/HOSTAGE/BUCKET参考")
    group = next((group for group in result["action_groups"] if group["name"] == group_name), None)
    if group is None:
        group = {"name": group_name, "points": []}
        result["action_groups"].append(group)
    group["points"].append({"kind": kind, "positions": {str(k): v for k, v in sorted(values.items())},
                            "time_ms": time_ms, "guard_ms": guard_ms, **source})
    if profile:
        result["references"][profile] = {"positions": [values[i] for i in range(3)],
                                         "confirmed": False, "note": "已串口采集，待重复返回和任务验收", **source}
    return result


def wizard(args):
    path = Path(args.config)
    config = load_config(path) if path.exists() else (current_example() if args.current_example else new_installation())
    print("机械臂录入向导：三平面关节000/001/002，夹爪003，所有长度mm、角度deg。")
    print("输入回车保留已有值；新机械安装的空值必须测量后填入。Ctrl+C退出，已保存项保留。")
    if args.current_example:
        print("当前示例包含旧HOSTAGE姿态和估计包络，请按本机重新录入/验收。")
    while True:
        print("\n1 机械尺寸  2 关节限位/标定  3 夹爪  4 障碍物  5 速度/时间")
        print("6 自动录动作点/参考  7 校验  8 导出预览  9 应用配置  0 保存退出")
        choice = input("选择: ").strip()
        try:
            if choice == "0":
                save_config(path, config)
                print(f"已保存 {path}")
                return
            if choice == "1":
                config["name"] = input(f"安装名(英文/数字/下划线) [{config['name']}]: ").strip() or config["name"]
                print("L1=000轴心到001轴心；L2=001轴心到002轴心。")
                print("工具x/z是腕部坐标内002轴心到实际抓点的偏移，不是世界坐标。")
                print("所有关节角为0时各杆沿+X；工具方向偏角=夹口方向减腕X方向。")
                for key in config["geometry"]:
                    config["geometry"][key] = ask(key, config["geometry"][key])
            elif choice == "2":
                index = ask("关节编号0/1/2", kind=int)
                if index not in (0, 1, 2):
                    raise ConfigError("只能选0/1/2")
                joint = config["joints"][index]
                print("000角相对+X；001/002角相对上一杆。限位按P数值大小填写，不自动扫限位。")
                for key in ("min_pwm", "max_pwm"):
                    if yes(f"将关节调到 {key}，自动查询这个边界P？"):
                        values, _ = connect_capture(args.port, [index], args.baud, args.timeout)
                        joint[key] = values[index]
                    else:
                        joint[key] = ask(key, joint[key], int)
                for n, point in enumerate(joint["calibration"]):
                    print(f"标定点{n+1}：先调到你知道机械角度的姿态。")
                    if yes("自动读取该标定点P？"):
                        values, _ = connect_capture(args.port, [index], args.baud, args.timeout)
                        point["pwm"] = values[index]
                    else:
                        point["pwm"] = ask("pwm", point["pwm"], int)
                    point["angle_deg"] = ask("该关节的实际标定角deg", point["angle_deg"])
            elif choice == "3":
                for key in ("min_pwm", "max_pwm", "close_pwm", "open_pwm"):
                    grip = config["gripper"]
                    if yes(f"调好夹爪 {key} 后自动读003？"):
                        values, _ = connect_capture(args.port, [3], args.baud, args.timeout)
                        grip[key] = values[3]
                    else:
                        grip[key] = ask(key, grip[key], int)
            elif choice == "4":
                c = config["collision"]
                c["enabled"] = yes("启用单箱体碰撞模型？", c["enabled"])
                if c["enabled"]:
                    print("以000轴心为原点，X向抓物为正，Z向上；Y=0为机械臂平面。")
                    for key in ("box_min_mm", "box_max_mm", "radii_mm"):
                        for index in range(3):
                            c[key][index] = ask(f"{key}[{'XYZ'[index] if key != 'radii_mm' else index}]", c[key][index])
                    c["clearance_mm"] = ask("clearance_mm", c["clearance_mm"])
                    c["sweep_resolution_mm"] = ask("sweep_resolution_mm", c["sweep_resolution_mm"])
                    c["envelope_estimated"] = yes("包络半径是否仍含估计？", c["envelope_estimated"])
            elif choice == "5":
                for section in ("trim", "timing"):
                    for key, value in config[section].items():
                        config[section][key] = ask(key, value, int if key.endswith("_ms") else float)
            elif choice == "6":
                kind = "gripper" if input("平面三关节/夹爪步骤 [p/g，默认p]: ").strip().lower() == "g" else "planar"
                group = input("动作组英文名，输入已有名称则追加步骤: ").strip()
                profile = input("同时设为哪个微调参考 BALL/HOSTAGE/BUCKET？留空仅录动作点: ").strip().upper() or None
                # Validate group/profile before opening any port.
                if not NAME_PATTERN.fullmatch(group) or profile not in (None, *PROFILES) or (profile and kind != "planar"):
                    raise ConfigError("动作组名称或参考类型无效")
                t = config["timing"]
                duration = ask("动作时间ms", t["reference_move_ms"] if kind == "planar" else t["grip_move_ms"], int)
                guard = ask("动作后稳定等待ms", t["reference_guard_ms"] if kind == "planar" else t["grip_guard_ms"], int)
                values, source = connect_capture(args.port, [0, 1, 2] if kind == "planar" else [3], args.baud, args.timeout)
                config = capture_point(config, values, group, kind, duration, guard, source, profile)
            elif choice == "7":
                for warning in validate(config):
                    print("提醒: " + warning)
                print("配置数据校验通过；实机验收仍需记录。")
            elif choice == "8":
                export_config(config, args.output)
            elif choice == "9":
                validate(config)
                if yes("替换本工程三份安装配置（自动备份，不构建/烧录）？"):
                    apply_config(config, args.project)
            else:
                print("请选择0..9")
                continue
            save_config(path, config)
            print(f"配置已保存: {path}")
        except (ConfigError, CaptureError, OSError) as error:
            print(f"未完成: {error}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="机械臂安装配置与总线舵机动作录入；不发送运动/烧录命令")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("ports", help="列出串口，不连接")
    template = sub.add_parser("template", help="创建新安装空模板或明确的当前安装示例")
    template.add_argument("--current-example", action="store_true")
    for name in ("template", "wizard", "validate", "export", "apply", "record"):
        p = template if name == "template" else sub.add_parser(name)
        p.add_argument("--config", default=str(DEFAULT_CONFIG))
        if name in ("wizard", "export"):
            p.add_argument("--output", default=str(ROOT / "build-local" / "arm_setup" / "generated"))
        if name in ("wizard", "apply"):
            p.add_argument("--project", default=str(ROOT))
        if name in ("wizard", "record"):
            p.add_argument("--port")
            p.add_argument("--baud", type=int, default=115200)
            p.add_argument("--timeout", type=float, default=1.0)
        if name == "wizard":
            p.add_argument("--current-example", action="store_true", help="仅新建文件时填入当前v4.6示例；已有文件不覆盖")
    record = sub.choices["record"]
    record.add_argument("--group", required=True)
    record.add_argument("--kind", choices=("planar", "gripper"), default="planar")
    record.add_argument("--profile", choices=PROFILES)
    record.add_argument("--time-ms", type=int, default=2000)
    record.add_argument("--guard-ms", type=int, default=300)
    args = parser.parse_args(argv)
    try:
        if args.command == "ports":
            for item in list_ports():
                print(f"{item['device']}: {item['description']}")
        elif args.command == "template":
            save_config(args.config, current_example() if args.current_example else new_installation(), replace=False)
            print(f"已创建 {args.config}")
        elif args.command == "wizard":
            wizard(args)
        else:
            config = load_config(args.config)
            if args.command == "record":
                if not NAME_PATTERN.fullmatch(args.group) or (args.profile and args.kind != "planar"):
                    raise ConfigError("动作组名称或参考类型无效")
                number(args.time_ms, "动作时间", 1, 9999, integer=True)
                number(args.guard_ms, "稳定等待", 0, 60000, integer=True)
                values, source = connect_capture(args.port, [0, 1, 2] if args.kind == "planar" else [3], args.baud, args.timeout)
                config = capture_point(config, values, args.group, args.kind, args.time_ms, args.guard_ms, source, args.profile)
                save_config(args.config, config)
                print(f"已追加到 {args.group}；参考映射={args.profile or '无'}")
            elif args.command == "validate":
                for warning in validate(config):
                    print("提醒: " + warning)
                print("配置数据校验通过")
            elif args.command == "export":
                export_config(config, args.output)
            elif args.command == "apply":
                apply_config(config, args.project)
        return 0
    except (ConfigError, CaptureError, OSError) as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print("\n已退出；本次尚未提交的录入不保存。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
