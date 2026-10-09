#!/usr/bin/env python3
"""Compare actual firmware MDG1 serializer against independently packed wire bytes."""
from pathlib import Path
import binascii
import struct
import subprocess
import tempfile

from test_motor_communication import HAL, ROOT, function


def main():
    source = (ROOT / "USB_DEVICE/App/usbd_cdc_if.c").read_text()
    # The extraction helper expects static; linkage does not affect the wire format.
    source = source.replace("\nuint8_t USB_CDC_SendMotorsTestState(",
                            "\nstatic uint8_t USB_CDC_SendMotorsTestState(")
    header = (ROOT / "USB_DEVICE/App/usbd_cdc_if.h").read_text()
    begin = header.index("typedef struct", header.index("/* 0x05 command"))
    end = header.index("/* USER CODE END EXPORTED_TYPES */", begin)
    serializer = function(source, "USB_CDC_Crc16Ccitt") + "\n" + function(source, "USB_CDC_SendMotorsTestState")
    fixture = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "service_types.h"
#define APP_TX_DATA_SIZE 2048
#define USB_CDC_HEADER_SIZE 10
#define USB_CDC_VERSION 2
#define USB_CDC_MSG_MOTORS_STATE 6
#define USBD_BUSY 1
#define USBD_FAIL 2
static uint8_t tx_frame[APP_TX_DATA_SIZE], tx_busy, cdc_result;
static uint16_t sent_length;
static uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length)
{ (void)data; sent_length = length; return cdc_result; }
#include "serializer.c"
int main(int argc, char **argv)
{
    assert(argc == 2);
    usb_cdc_motors_test_state_t s = {0};
    s.timestamp_ms = 214295; s.command_seq = 65534; s.flags = 7;
    s.rx_lost_count = 11; s.tx_error_count = 12;
    s.command_age_ms = 3; s.max_control_gap_ms = 4;
    s.kp_nm_per_rad = 8.25f; s.kd_nms_per_rad = 0.5f;
    s.configured_max_torque_nm = 2.0f; s.imu_status = 0x8003;
    for (unsigned b = 0; b < 2; ++b)
    {
        s.buses[b].psr = 0x707 + b; s.buses[b].ecr = 0x10203 + b;
        s.buses[b].hal_error = b + 9; s.buses[b].valid = 1;
    }
    for (unsigned i = 0; i < 6; ++i)
    {
        usb_cdc_motor_diagnostic_t *m = &s.motors[i];
        m->target = i + 0.25f; m->position = -(float)i - 0.5f;
        m->velocity = i + 1.5f; m->torque = i + 0.75f;
        m->age_ms = i + 1; m->accept_count = i + 100; m->suspect_count = i;
        m->valid = 1; m->fault = i;
        usb_cdc_motor_trace_t *d = &s.traces[i];
        d->last_fault_ms = i + 200; d->fault_age_ms = i + 2;
        d->fault_valid = 1; d->mode = 10; d->temperature = -5 + (int)i;
        for (unsigned j = 0; j < 5; ++j) d->pd_codes[j] = (int)i * 10 + (int)j - 20;
        d->tx_valid = 1;
        motor_feedback_trace_t *traces[] = {&d->last_rx, &d->last_reject, &d->peak_rx};
        for (unsigned j = 0; j < 3; ++j)
        {
            motor_feedback_trace_t *t = traces[j];
            t->timestamp_ms = 300 + i * 3 + j; t->can_id = ((i % 3) + 1) << 8;
            t->previous_age_ms = j ? j : UINT32_MAX;
            t->position = i * 3 + j + 0.25f; t->velocity = -(float)(i * 3 + j) - 0.5f;
            t->torque = 0.75f; t->previous_position = -0.25f; t->previous_velocity = 1.5f;
            t->reason = j + 1; t->format = 4; t->data_length = 24; t->valid = 1;
            for (unsigned k = 0; k < 24; ++k) t->raw[k] = i * 32 + j + k;
        }
    }
    assert(USB_CDC_SendMotorsTestState(&s) == USBD_OK);
    assert(sent_length == 1496 && tx_busy == 1);
    FILE *f = fopen(argv[1], "wb"); assert(f);
    assert(fwrite(tx_frame, 1, sent_length, f) == sent_length); fclose(f);
    assert(USB_CDC_SendMotorsTestState(&s) == USBD_BUSY);
    tx_busy = 0; cdc_result = USBD_BUSY;
    assert(USB_CDC_SendMotorsTestState(&s) == USBD_BUSY && tx_busy == 0);
    return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix="motor_wire_") as directory:
        tmp = Path(directory)
        (tmp / "stm32h7xx_hal.h").write_text(HAL)
        for name in ("main.h", "stm32h7xx_hal_fdcan.h", "stm32h723xx.h"):
            (tmp / name).write_text('#include "stm32h7xx_hal.h"\n')
        (tmp / "service_types.h").write_text(header[begin:end])
        (tmp / "serializer.c").write_text(serializer)
        (tmp / "fixture.c").write_text(fixture)
        includes = [tmp, *[ROOT / "src" / name for name in
                          ("motor", "convert", "livelybot_fdcan")], ROOT / "App/my_fdcan"]
        executable = tmp / "wire"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        *[f"-I{p}" for p in includes], str(tmp / "fixture.c"),
                        "-o", str(executable)], check=True)
        output = tmp / "wire.bin"
        subprocess.run([str(executable), str(output)], check=True)
        actual = output.read_bytes()
        payload = struct.pack("<IHHII", 214295, 65534, 7, 11, 12)
        for i in range(6):
            payload += struct.pack("<4f3IBB", i + .25, -i - .5, i + 1.5, i + .75,
                                   i + 1, i + 100, i, 1, i)
        payload += struct.pack("<4sHHII3fI", b"MDG1", 1, 1288, 3, 4, 8.25, .5, 2, 0x8003)
        for b in range(2):
            payload += struct.pack("<4I", 0x707 + b, 0x10203 + b, b + 9, 1)
        for i in range(6):
            payload += struct.pack("<IIBBbB5hH", i + 200, i + 2, 1, 10, -5 + i, 0,
                                   *[i * 10 + j - 20 for j in range(5)], 1)
            for j in range(3):
                payload += struct.pack("<III5f4B24s", 300 + i * 3 + j, ((i % 3) + 1) << 8,
                                       j or 0xffffffff, i * 3 + j + .25, -i * 3 - j - .5,
                                       .75, -.25, 1.5, j + 1, 4, 24, 1,
                                       bytes(i * 32 + j + k for k in range(24)))
        assert len(payload) == 1484
        expected = struct.pack("<4sBBHH", b"RB32", 2, 6, 0, len(payload)) + payload
        expected += struct.pack("<H", binascii.crc_hqx(expected, 0xffff))
        assert actual == expected, "Actual firmware bytes differ from MDG1 contract"
    print("PASS: actual USB serializer, 1484-byte MDG1 field offsets, CRC and busy handling")
    return actual


if __name__ == "__main__":
    main()
