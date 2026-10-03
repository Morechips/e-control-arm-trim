"""Export the generic C11 planner; no HAL, project parameters or transport."""
from pathlib import Path
import hashlib
import json
import zipfile


def main():
    root = Path(__file__).resolve().parent.parent
    files = []
    for module in ("arm_kinematics", "arm_collision", "arm_trim"):
        files.extend((f"Core/Inc/{module}.h", f"Core/Src/{module}.c"))
    files.extend(("examples/arm_trim_portable/demo.c",
                  "examples/arm_trim_portable/run.ps1",
                  "examples/arm_trim_portable/README.md"))
    output = root / "firmware_direct" / "arm-trim-core-v4.6.zip"
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest = {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                for name in files}
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as package:
        for name in files:
            package.write(root / name, name)
        package.writestr("SHA256.json", json.dumps(manifest, indent=2))
    print(output)


if __name__ == "__main__":
    main()
