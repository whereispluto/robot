/* Compiled by test_motor_communication.py with the actual service functions. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor.c"
#include "motor_many.h"
#include "motor_control.h"
#include "service_types.h"

FDCAN_HandleTypeDef hfdcan1 = {FDCAN1}, hfdcan2 = {FDCAN2};
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t f,
                          FDCAN_RxHeaderTypeDef *r, uint8_t *d) { return 1; }
uint16_t get_fdcan_data_size(uint32_t dlc) { return (uint16_t)dlc; }
uint32_t fdcan_get_tx_error_count(void) { return 0; }

static uint8_t legacy[8] = {0x27, 0x01, 1, 0, 2, 0, 3, 0};
/* Full int16 response, padded to a valid CAN FD DLC of 16 bytes. */
static uint8_t full[16] = {0x24, 4, 0, 10, 0, 1, 0, 2, 0, 3, 0,
                          0x21, 0x0f, 0, 0x50, 0x50};
static unsigned fault_queries, control_frames;
static uint8_t emulate, drop_fault_reply, injected_fault[2][3];
static int16_t last_kp[2][3];

void fdcan_send(FDCAN_HandleTypeDef *h, uint32_t id, uint8_t *data, uint16_t size)
{
    const unsigned port = h == &hfdcan1 ? 0 : 1;
    assert(emulate);
    if (id == 0x80b0)
    {
        /* The PD command keeps the existing, universally supported query. */
        assert(size == 64 && data[62] == 0x17 && data[63] == 0x01);
        ++control_frames;
        for (uint8_t motor = 1; motor <= 3; ++motor)
        {
            memcpy(&last_kp[port][motor - 1], data + (motor - 1) * 10 + 6, 2);
            motor_process_state(h, motor, legacy, sizeof legacy);
        }
    }
    else
    {
        static const uint8_t expected[] = {0x14, 4, 0, 0x11, 0x0f};
        assert(id >= 0x8001 && id <= 0x8003);
        assert(size == sizeof expected && memcmp(data, expected, size) == 0);
        ++fault_queries;
        if (!drop_fault_reply)
        {
            full[13] = injected_fault[port][(id & 0xff) - 1];
            motor_process_state(h, id & 0xff, full, sizeof full);
        }
    }
}

static usb_cdc_motors_test_command_t g_motors_test_command;
static uint32_t g_motors_test_tick;
static uint16_t g_motors_test_flags, g_motors_test_applied_seq;
static float g_motors_test_target[6];
static usb_cdc_motors_test_state_t telemetry;
static uint8_t USB_CDC_SendMotorsTestState(const usb_cdc_motors_test_state_t *s)
{ telemetry = *s; return 0; }
#include "service.c"

static void test_parser(void)
{
    for (unsigned port = 0; port < 2; ++port)
    for (uint8_t id = 1; id <= 3; ++id)
    {
        FDCAN_HandleTypeDef *h = port ? &hfdcan2 : &hfdcan1;
        motor_state_s *s = motor_get_state(port ? PORT2 : PORT1, id);
        tick = 100;
        motor_process_state(h, id, legacy, sizeof legacy);
        assert(s->valid && s->accept_count == 1 && !s->fault_valid);
        assert(motor_get_fresh_fault(s, tick, 60) == MOTOR_FAULT_UNKNOWN);
        int reversed = (!port && id == 1) || (port && id >= 2);
        assert(reversed ? s->position < 0 : s->position > 0);
        assert(reversed ? s->velocity < 0 : s->velocity > 0);
        full[13] = 0;
        motor_process_state(h, id, full, sizeof full);
        assert(s->accept_count == 2 && motor_get_fresh_fault(s, tick, 60) == 0);
        full[13] = 1;
        tick = 102;
        motor_process_state(h, id, full, sizeof full);
        tick = 104;
        motor_process_state(h, id, legacy, sizeof legacy);
        assert(s->fault == 1 && s->last_fault_ms == 102);
        assert(motor_get_fresh_fault(s, 162, 60) == 1);
        assert(motor_get_fresh_fault(s, 163, 60) == MOTOR_FAULT_UNKNOWN);

        uint8_t compact[8] = {39, 0x40, 1, 0, 2, 0, 3, 0};
        motor_process_state(h, id, compact, sizeof compact);
        assert(s->temp == 39 && s->fault == 0);
        compact[1] = 0x41;
        motor_process_state(h, id, compact, sizeof compact);
        assert(s->fault == 1);
        compact[1] = 0x80;
        uint32_t count = s->accept_count;
        motor_process_state(h, id, compact, sizeof compact);
        assert(s->accept_count == count && s->suspect_count == 1);
        assert(s->fault == 1);
        full[5] = 0; full[6] = 0x80; full[13] = 35;
        motor_process_state(h, id, full, sizeof full);
        assert(s->accept_count == count && s->suspect_count == 2 && s->fault == 35);
        full[5] = 1; full[6] = 0; full[13] = 0;
        motor_process_state(h, id, full, 7); /* Truncated full-state response. */
        assert(s->fault == 35);
        tick = UINT32_MAX - 10;
        motor_process_state(h, id, full, sizeof full);
        assert(motor_get_fresh_fault(s, 49, 60) == 0);
        assert(motor_get_fresh_fault(s, 50, 60) == MOTOR_FAULT_UNKNOWN);
        s->valid = s->fault_valid = 0;
    }
}

static void step(uint32_t now)
{
    tick = g_motors_test_tick = now;
    USB_ApplyMotorsTest();
    USB_SendMotorsTestState();
}

static void test_service(void)
{
    emulate = 1;
    g_motors_test_command.flags = 1;
    g_motors_test_command.kp = 0.5f;
    g_motors_test_command.seq = 123;
    step(1000);
    assert(!(telemetry.flags & 4)); /* Wait for actual fault feedback. */
    assert(fault_queries == 6 && control_frames == 2);
    step(1002);
    assert(telemetry.flags & 4);
    assert(telemetry.command_seq == 123);
    for (unsigned j = 0; j < 6; ++j)
        assert(telemetry.motors[j].valid && telemetry.motors[j].fault == 0);
    for (uint32_t now = 1004; now <= 1018; now += 2) step(now);
    assert(fault_queries == 6); /* No 500 Hz burst of diagnostic queries. */
    step(1020);
    assert(fault_queries == 12);
    injected_fault[1][1] = 1;
    step(1040); step(1042);
    assert(!(telemetry.flags & 4) && telemetry.motors[4].fault == 1);
    for (unsigned port = 0; port < 2; ++port)
        for (unsigned id = 0; id < 3; ++id)
            assert(last_kp[port][id] == 0);
    injected_fault[1][1] = 0;
    step(1060); step(1062);
    assert(telemetry.flags & 4);
    drop_fault_reply = 1;
    for (uint32_t now = 1064; now <= 1122; now += 2) step(now);
    assert(!(telemetry.flags & 4));
    for (unsigned j = 0; j < 6; ++j)
    {
        assert(telemetry.motors[j].valid && telemetry.motors[j].age_ms == 0);
        assert(telemetry.motors[j].fault == MOTOR_FAULT_UNKNOWN);
    }
    drop_fault_reply = 0;
    step(1140); step(1142);
    assert(telemetry.flags & 4);
    tick = 1244; /* Fresh CAN data cannot override the PC command timeout. */
    USB_ApplyMotorsTest();
    assert(!(g_motors_test_flags & 4));
}

int main(void)
{
    test_parser();
    test_service();
    puts("PASS: actual parser, CAN queries, six-motor enable/stop, feedback and command timeouts");
}
