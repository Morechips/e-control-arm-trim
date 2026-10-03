"""Configuration and generated-C integration checks; no serial hardware."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import arm_setup as setup


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.config = setup.current_example()

    def rejected(self, change):
        change(self.config)
        with self.assertRaises(setup.ConfigError):
            setup.validate(self.config)

    def test_current_example_and_empty_new_installation(self):
        self.assertTrue(setup.validate(self.config))
        with self.assertRaises(setup.ConfigError):
            setup.validate(setup.new_installation())

    def test_non_finite_bool_and_non_integer(self):
        for value in (float("nan"), float("inf"), True, None, "84.75", 1e100, 10 ** 500):
            with self.subTest(value=value), self.assertRaises(setup.ConfigError):
                cfg = setup.current_example()
                cfg["geometry"]["link_2_mm"] = value
                setup.validate(cfg)
        self.rejected(lambda cfg: cfg["joints"][0].update(min_pwm=915.5))

    def test_ids_must_be_integer(self):
        for value in (0.0, False, "0"):
            with self.subTest(value=value), self.assertRaises(setup.ConfigError):
                cfg = setup.current_example()
                cfg["joints"][0]["id"] = value
                setup.validate(cfg)

    def test_disabled_new_box_uses_inactive_placeholders(self):
        cfg = setup.current_example()
        cfg["collision"] = setup.new_installation()["collision"]
        setup.validate(cfg)
        self.assertIn("#define ARM_COLLISION_PROJECT_ENABLED 0", setup.headers(cfg)["arm_collision_config.h"])
        cfg["collision"]["enabled"] = True
        with self.assertRaises(setup.ConfigError):
            setup.validate(cfg)

    def test_reversed_limits_and_outside_calibration(self):
        self.rejected(lambda cfg: cfg["joints"][0].update(min_pwm=1800, max_pwm=915))
        self.config = setup.current_example()
        self.rejected(lambda cfg: cfg["joints"][1]["calibration"][1].update(pwm=2600))

    def test_two_point_angles_can_reverse_but_cannot_equal(self):
        self.config["joints"][0]["calibration"][1]["angle_deg"] = -30
        setup.validate(self.config)
        self.rejected(lambda cfg: cfg["joints"][0]["calibration"][1].update(angle_deg=0))

    def test_reference_never_contains_gripper(self):
        self.rejected(lambda cfg: cfg["references"]["BALL"].update(positions=[1356, 1850, 698, 500]))

    def test_trajectory_caps(self):
        for key, value in (("search_mm", 76), ("speed_mm_s", 11),
                           ("acceleration_mm_s2", 21), ("min_offset_mm", -76),
                           ("update_period_ms", 20), ("max_segment_mm", 3)):
            with self.subTest(key=key), self.assertRaises(setup.ConfigError):
                cfg = setup.current_example()
                cfg["trim"][key] = value
                setup.validate(cfg)

    def test_bad_box_and_gripper(self):
        self.rejected(lambda cfg: cfg["collision"].update(box_min_mm=[0, 0, 0]))
        self.config = setup.current_example()
        self.rejected(lambda cfg: cfg["gripper"].update(min_pwm=900))

    def test_capture_atomic_and_steps_preserved(self):
        original = copy.deepcopy(self.config)
        with self.assertRaises(setup.ConfigError):
            setup.capture_point(self.config, {0: 1356, 1: 1850}, "take_ball", "planar", 2000, 300, {})
        self.assertEqual(original, self.config)
        cfg = setup.capture_point(self.config, {0: 1356, 1: 1850, 2: 698}, "take_ball", "planar", 2000, 300, {}, "BALL")
        cfg = setup.capture_point(cfg, {3: 500}, "take_ball", "gripper", 1500, 300, {})
        cfg = setup.capture_point(cfg, {0: 1400, 1: 1900, 2: 700}, "take_ball", "planar", 1000, 400, {})
        setup.validate(cfg)
        self.assertEqual(["planar", "gripper", "planar"], [p["kind"] for p in cfg["action_groups"][0]["points"]])
        self.assertEqual([1356, 1850, 698], cfg["references"]["BALL"]["positions"])
        self.assertFalse(cfg["references"]["BALL"]["confirmed"])
        self.assertNotIn("3", cfg["action_groups"][0]["points"][0]["positions"])
        self.assertEqual(original, self.config)

    def test_reference_capture_limits_and_bad_group(self):
        for values, name, kind, profile in (({0: 900, 1: 1850, 2: 698}, "take_ball", "planar", "BALL"),
                                            ({3: 500}, "take_ball", "gripper", "BALL"),
                                            ({3: 500}, 'bad"name', "gripper", None)):
            with self.assertRaises(setup.ConfigError):
                setup.capture_point(self.config, values, name, kind, 2000, 300, {}, profile)

    def test_save_backup_and_no_template_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "installation.json"
            setup.save_config(path, self.config, replace=False)
            with self.assertRaises(setup.ConfigError):
                setup.save_config(path, setup.new_installation(), replace=False)
            cfg = copy.deepcopy(self.config)
            cfg["name"] = "updated"
            setup.save_config(path, cfg)
            self.assertEqual("updated", setup.load_config(path)["name"])
            self.assertEqual(self.config, setup.load_config(str(path) + ".bak"))

    def test_export_separates_gripper_and_rejects_invalid_before_write(self):
        cfg = setup.capture_point(self.config, {0: 1356, 1: 1850, 2: 698}, "take_ball", "planar", 2000, 300, {})
        cfg = setup.capture_point(cfg, {3: 500}, "take_ball", "gripper", 1500, 300, {})
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "generated"
            setup.export_config(cfg, output)
            text = (output / "action_commands.txt").read_text(encoding="utf-8")
            frames = [line for line in text.splitlines() if line.startswith("{")]
            self.assertNotIn("#003", frames[0])
            self.assertEqual("{#003P0500T1500!}", frames[1])
            self.assertIn("7U", (output / "arm_recorded_actions.h").read_text())
            cfg["references"]["BALL"]["positions"] = [0, 1850, 698]
            before = (output / "arm_trim_project_config.h").read_bytes()
            with self.assertRaises(setup.ConfigError):
                setup.export_config(cfg, output)
            self.assertEqual(before, (output / "arm_trim_project_config.h").read_bytes())

    def test_apply_compatible_project_and_backup(self):
        with tempfile.TemporaryDirectory() as folder:
            project = Path(folder)
            inc, src = project / "Core" / "Inc", project / "Core" / "Src"
            inc.mkdir(parents=True)
            src.mkdir(parents=True)
            for name in setup.headers(self.config):
                (inc / name).write_text("original " + name)
            (src / "arm_trim_project.c").write_text("ARM_TRIM_PROJECT_Q0_A_DEG")
            (src / "arm_trim_input.c").write_text("ARM_TRIM_GRIPPER_MIN_P")
            (inc / "arm_trim_service.h").write_text("grip_min_pwm")
            setup.apply_config(self.config, project)
            backups = list((project / "build-local" / "arm_setup").glob("backup-*"))
            self.assertEqual(1, len(backups))
            self.assertEqual("original arm_trim_project_config.h", (backups[0] / "arm_trim_project_config.h").read_text())
            self.assertIn("84.75f", (inc / "arm_trim_project_config.h").read_text())
            # Source/config outside the three generated headers is untouched.
            self.assertEqual("grip_min_pwm", (inc / "arm_trim_service.h").read_text())

    def test_apply_rejects_old_project_without_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            project = Path(folder)
            inc, src = project / "Core" / "Inc", project / "Core" / "Src"
            inc.mkdir(parents=True)
            src.mkdir(parents=True)
            for name in setup.headers(self.config):
                (inc / name).write_text("original")
            (src / "arm_trim_project.c").write_text("old adapter")
            (src / "arm_trim_input.c").write_text("old input")
            (inc / "arm_trim_service.h").write_text("old service")
            with self.assertRaises(setup.ConfigError):
                setup.apply_config(self.config, project)
            self.assertEqual("original", (inc / "arm_trim_project_config.h").read_text())
            self.assertFalse((project / "build-local").exists())

    def test_export_rejects_other_projects_and_foreign_headers(self):
        with tempfile.TemporaryDirectory() as folder:
            inc = Path(folder) / "other" / "Core" / "Inc"
            inc.mkdir(parents=True)
            original = inc / "arm_trim_project_config.h"
            original.write_text("original")
            with self.assertRaises(setup.ConfigError):
                setup.export_config(self.config, inc)
            self.assertEqual("original", original.read_text())
            foreign = Path(folder) / "foreign"
            foreign.mkdir()
            (foreign / "arm_trim_project_config.h").write_text("original")
            with self.assertRaises(setup.ConfigError):
                setup.export_config(self.config, foreign)

    def test_apply_interrupt_rolls_back_all_headers(self):
        with tempfile.TemporaryDirectory() as folder:
            project = Path(folder)
            inc, src = project / "Core" / "Inc", project / "Core" / "Src"
            inc.mkdir(parents=True)
            src.mkdir(parents=True)
            for name in setup.headers(self.config):
                (inc / name).write_text("original " + name)
            (src / "arm_trim_project.c").write_text("ARM_TRIM_PROJECT_Q0_A_DEG")
            (src / "arm_trim_input.c").write_text("ARM_TRIM_GRIPPER_MIN_P")
            (inc / "arm_trim_service.h").write_text("grip_min_pwm")
            original_replace = Path.replace
            count = 0
            def interrupt_replace(path, target):
                nonlocal count
                count += 1
                if count == 2:
                    raise KeyboardInterrupt()
                return original_replace(path, target)
            with patch.object(Path, "replace", interrupt_replace), self.assertRaises(KeyboardInterrupt):
                setup.apply_config(self.config, project)
            for name in setup.headers(self.config):
                self.assertEqual("original " + name, (inc / name).read_text())
            self.assertFalse(list(inc.glob("*.setup.tmp")))


class GeneratedCIntegrationTests(unittest.TestCase):
    def test_exported_geometry_calibration_trajectory_and_action_table(self):
        gcc = shutil.which("gcc") or (r"D:\c++\MinGW\bin\gcc.exe" if Path(r"D:\c++\MinGW\bin\gcc.exe").is_file() else None)
        self.assertIsNotNone(gcc, "host GCC is required for generated-C integration test")
        cfg = setup.current_example()
        cfg["geometry"]["link_2_mm"] = 100.25
        cfg["joints"][1]["calibration"][1]["angle_deg"] = -60
        cfg["trim"].update(speed_mm_s=4, acceleration_mm_s2=8, update_period_ms=60)
        cfg = setup.capture_point(cfg, {0: 1356, 1: 1850, 2: 698}, "take_ball", "planar", 2000, 300, {})
        cfg = setup.capture_point(cfg, {3: 500}, "take_ball", "gripper", 1500, 300, {})
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            setup.export_config(cfg, output)
            probe = output / "probe.c"
            probe.write_text('''#include "arm_trim_project.h"
#include "arm_recorded_actions.h"
#include <math.h>
int main(void) {
    ArmTrimConfig_t c;
    ArmTrimProject_DefaultConfig(&c);
    if (fabsf(c.geometry.link_2_mm - 100.25f) > 0.001f) return 1;
    if (fabsf(c.calibration[1].angle_b_rad + 1.04719755f) > 0.00001f) return 2;
    if (c.speed_mm_s != 4.0f || c.acceleration_mm_s2 != 8.0f || c.update_period_ms != 60U) return 3;
    if (ARM_RECORDED_GROUP_COUNT != 1U || arm_recorded_groups[0].count != 2U) return 4;
    if (arm_recorded_groups[0].steps[0].mask != 7U || arm_recorded_groups[0].steps[1].mask != 8U) return 5;
    return 0;
}
''')
            executable = output / ("probe.exe" if os.name == "nt" else "probe")
            command = [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                       "-I" + str(output), "-I" + str(ROOT / "Core" / "Inc"), str(probe),
                       str(ROOT / "Core" / "Src" / "arm_trim_project.c"),
                       str(ROOT / "Core" / "Src" / "arm_trim.c"),
                       str(ROOT / "Core" / "Src" / "arm_kinematics.c"),
                       str(ROOT / "Core" / "Src" / "arm_collision.c"), "-lm", "-o", str(executable)]
            subprocess.run(command, check=True, capture_output=True)
            subprocess.run([str(executable)], check=True, capture_output=True)


if __name__ == "__main__":
    unittest.main()
