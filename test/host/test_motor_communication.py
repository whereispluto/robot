#!/usr/bin/env python3
"""Run the real CAN parser and six-motor service with a fake HAL, offline."""

from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HAL = """
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier, DataLength, IdType, RxFrameType; } FDCAN_RxHeaderTypeDef;
#define FDCAN1 ((void *)1)
#define FDCAN2 ((void *)2)
#define FDCAN_RX_FIFO0 0
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 1
#define HAL_OK 0
uint32_t HAL_GetTick(void);
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
"""


def function(source, name):
    """Extract the unchanged function definition, excluding its prototype."""
    match = re.search(r"^static [^\n]+\b" + name + r"\([^;]+?\)\s*\{", source, re.M)
    if match is None:
        raise ValueError(f"Function definition not found: {name}")
    depth = 1
    end = match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def main():
    main_source = (ROOT / "Core/Src/main.c").read_text()
    header = (ROOT / "USB_DEVICE/App/usbd_cdc_if.h").read_text()
    types_start = header.index("typedef struct", header.index("/* 0x05 command"))
    types_end = header.index("/* USER CODE END EXPORTED_TYPES */", types_start)
    definitions = "\n".join(
        line for line in main_source.splitlines()
        if line.startswith("#define ") and any(name in line for name in (
            "USB_STATE_PERIOD_MS", "USB_GAIN_TEST_COMMAND_TIMEOUT_MS", "USB_TWO_PI",
            "MOTOR_FEEDBACK_MONITOR_TIMEOUT_MS", "ROBOT_JOINT_COUNT",
            "USB_CONTROL_TARGET_VELOCITY", "USB_CONTROL_FEEDFORWARD_TORQUE",
        ))
    )
    service = "\n".join(function(main_source, name) for name in (
        "USB_GetJointMotorState", "USB_SetGainTestMotor",
        "USB_ApplyMotorsTest", "USB_SendMotorsTestState",
    ))
    with tempfile.TemporaryDirectory(prefix="motor_communication_") as directory:
        tmp = Path(directory)
        (tmp / "stm32h7xx_hal.h").write_text(HAL)
        for name in ("main.h", "stm32h7xx_hal_fdcan.h", "stm32h723xx.h"):
            (tmp / name).write_text('#include "stm32h7xx_hal.h"\n')
        (tmp / "service_types.h").write_text(header[types_start:types_end] + definitions)
        (tmp / "service.c").write_text(service)
        includes = [tmp, ROOT / "src/motor", ROOT / "src/convert",
                    ROOT / "src/livelybot_fdcan", ROOT / "src/motor_many",
                    ROOT / "src/motor_control", ROOT / "App/my_fdcan"]
        sources = [ROOT / "test/host/test_motor_communication.c", *[
            ROOT / f"src/{name}/{name}.c"
            for name in ("convert", "motor_many", "motor_control", "livelybot_fdcan")
        ]]
        executable = tmp / "test_motor_communication"
        subprocess.run([
            "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-Wno-sign-compare",  # Existing SDK int loops compare with sizeof.
            *[f"-I{path}" for path in includes], *map(str, sources),
            "-lm", "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
