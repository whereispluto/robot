/* Compiled by test_motor_communication.py with the actual service functions. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor.c"
#include "motor_many.h"
#include "motor_control.h"
#include "service_types.h"

FDCAN_GlobalTypeDef fake_fdcan1 = {0x707, 0x1234}, fake_fdcan2 = {0x90, 0x5678};
FDCAN_HandleTypeDef hfdcan1 = {FDCAN1, 0x12}, hfdcan2 = {FDCAN2, 0x34};
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
static uint32_t g_motors_test_max_control_gap_ms;
static float g_configured_max_torque_nm = 2.0f;
static struct { uint16_t status; } g_imu_data = {0x8003};
static uint16_t g_motors_test_flags, g_motors_test_applied_seq;
static float g_motors_test_target[6];
static usb_cdc_motors_test_state_t telemetry;
static uint8_t usb_send_result;
static uint8_t USB_CDC_SendMotorsTestState(const usb_cdc_motors_test_state_t *s)
{ telemetry = *s; return usb_send_result; }
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

    /* Capture the actual configured command and packed SDK codes alongside
     * register-level bus diagnostics, independent of successful USB delivery. */
    g_motors_test_command.kp = 8.2f;
    g_motors_test_command.kd = 0.5f;
    g_motors_test_command.seq = 456;
    g_motors_test_tick = 1295;
    g_motors_test_max_control_gap_ms = 7;
    tick = 1300;
    USB_ApplyMotorsTest();
    assert(g_motors_test_flags & 4);
    uint8_t spike[8];
    memcpy(spike, legacy, sizeof spike);
    const int16_t spike_velocity = 5240;
    memcpy(spike + 4, &spike_velocity, sizeof spike_velocity);
    tick = 1302;
    motor_process_state(&hfdcan1, 1, spike, sizeof spike);
    const motor_feedback_trace_t pending_peak = motor_get_state(PORT1, 1)->peak_rx;
    assert(pending_peak.valid && fabsf(pending_peak.velocity + 471.6f) < 0.001f);

    tick = 1306;
    usb_send_result = 1; /* Queue busy/failure must retain evidence for retry. */
    USB_SendMotorsTestState();
    assert(telemetry.timestamp_ms == 1306U && telemetry.command_seq == 456);
    assert(telemetry.command_age_ms == 11U && telemetry.max_control_gap_ms == 7U);
    assert(telemetry.kp_nm_per_rad == 8.2f && telemetry.kd_nms_per_rad == 0.5f);
    assert(telemetry.configured_max_torque_nm == g_configured_max_torque_nm);
    assert(telemetry.imu_status == 0x8003);
    assert(telemetry.buses[0].valid && telemetry.buses[1].valid);
    assert(telemetry.buses[0].psr == 0x707 && telemetry.buses[0].ecr == 0x1234);
    assert(telemetry.buses[0].hal_error == 0x12);
    assert(telemetry.buses[1].psr == 0x90 && telemetry.buses[1].ecr == 0x5678);
    assert(telemetry.buses[1].hal_error == 0x34);
    for (uint8_t joint = 0; joint < 6; ++joint)
    {
        const port_t port = joint < 3 ? PORT1 : PORT2;
        const uint8_t id = joint % 3 + 1;
        motor_state_s *m = motor_get_state(port, id);
        const usb_cdc_motor_trace_t *trace = &telemetry.traces[joint];
        int16_t expected_codes[5];
        assert(motor_many_get_pd_codes(port, id, expected_codes));
        assert(trace->tx_valid && memcmp(trace->pd_codes, expected_codes,
                                         sizeof expected_codes) == 0);
        assert(trace->pd_codes[3] == 980 && trace->pd_codes[4] == 59);
        assert(trace->fault_valid && trace->last_fault_ms == 1300U);
        assert(trace->fault_age_ms == 6U && telemetry.motors[joint].fault == 0U);
        assert(trace->mode == m->mode && trace->temperature == m->temp);
        assert(memcmp(&trace->last_rx, &m->last_rx, sizeof m->last_rx) == 0);
        assert(memcmp(&trace->last_reject, &m->last_reject, sizeof m->last_reject) == 0);
        assert(trace->peak_rx.valid && m->peak_rx.valid);
    }
    assert(g_motors_test_max_control_gap_ms == 7U);
    assert(memcmp(&motor_get_state(PORT1, 1)->peak_rx, &pending_peak,
                  sizeof pending_peak) == 0);
    assert(memcmp(&telemetry.traces[0].peak_rx, &pending_peak,
                  sizeof pending_peak) == 0);

    /* Successful queueing must include the pending peak and gap, then clear
     * only the interval accumulators. Unknown fault age stays explicit. */
    motor_get_state(PORT2, 3)->fault_valid = 0U;
    usb_send_result = USBD_OK;
    USB_SendMotorsTestState();
    assert(telemetry.max_control_gap_ms == 7U && g_motors_test_max_control_gap_ms == 0U);
    assert(memcmp(&telemetry.traces[0].peak_rx, &pending_peak, sizeof pending_peak) == 0);
    assert(!telemetry.traces[5].fault_valid && telemetry.traces[5].fault_age_ms == UINT32_MAX);
    assert(telemetry.motors[5].fault == MOTOR_FAULT_UNKNOWN);
    for (uint8_t joint = 0; joint < 6; ++joint)
    {
        motor_state_s *m = motor_get_state(joint < 3 ? PORT1 : PORT2, joint % 3 + 1);
        assert(!m->peak_rx.valid);
        assert(memcmp(&telemetry.traces[joint].last_reject, &m->last_reject,
                      sizeof m->last_reject) == 0);
    }
}

static void test_diagnostic_capture(void)
{
    motor_state_s *s = motor_get_state(PORT1, 1);
    uint8_t sample[8];
    memcpy(sample, legacy, sizeof sample);
    s->valid = 0U;
    motor_clear_diagnostic_peaks();
    const uint32_t original_suspects = s->suspect_count;

    tick = 2000;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(s->last_rx.valid && s->last_rx.format == MOTOR_TRACE_FORMAT_LEGACY);
    assert(s->last_rx.reason == MOTOR_TRACE_REASON_NONE);
    assert(s->last_rx.timestamp_ms == tick && s->last_rx.can_id == 0x100);
    assert(s->last_rx.previous_age_ms == UINT32_MAX);
    assert(isnan(s->last_rx.previous_position) && isnan(s->last_rx.previous_velocity));
    assert(s->last_rx.data_length == sizeof sample);
    assert(memcmp(s->last_rx.raw, sample, sizeof sample) == 0);

    /* The observed -471.6 deg/s spike is accepted by the existing 1500 deg/s
     * firmware limit. Preserve it even when later frames return to normal. */
    int16_t velocity = 5240;
    memcpy(sample + 4, &velocity, sizeof velocity);
    tick = 2002;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(fabsf(s->last_rx.velocity + 471.6f) < 0.001f);
    assert(s->last_rx.previous_age_ms == 2U);
    assert(fabsf(s->last_rx.previous_velocity + 0.18f) < 0.001f);
    assert(s->peak_rx.valid && s->peak_rx.timestamp_ms == 2002U);
    assert(s->suspect_count == original_suspects);
    velocity = 2;
    memcpy(sample + 4, &velocity, sizeof velocity);
    tick = 2004;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(s->last_rx.timestamp_ms == 2004U && s->peak_rx.timestamp_ms == 2002U);

    /* An out-of-range candidate must be preserved without changing accepted
     * motion state or weakening the existing rejection threshold. */
    uint32_t accepted = s->accept_count;
    velocity = 20000; /* J1 direction correction: -1800 deg/s. */
    memcpy(sample + 4, &velocity, sizeof velocity);
    tick = 2006;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(s->accept_count == accepted && s->last_update_ms == 2004U);
    assert(s->suspect_count == original_suspects + 1U);
    assert(s->last_reject.reason == MOTOR_TRACE_REASON_VELOCITY_LIMIT);
    assert(s->last_reject.previous_age_ms == 2U);
    assert(fabsf(s->last_reject.velocity + 1800.0f) < 0.001f);
    assert(memcmp(s->last_reject.raw, sample, sizeof sample) == 0);
    assert(s->peak_rx.timestamp_ms == 2006U);
    assert(s->peak_rx.reason == MOTOR_TRACE_REASON_VELOCITY_LIMIT);
    const motor_feedback_trace_t rejected_speed = s->last_reject;
    motor_clear_diagnostic_peaks();
    assert(!s->peak_rx.valid && s->last_rx.valid);
    assert(memcmp(&s->last_reject, &rejected_speed, sizeof rejected_speed) == 0);
    assert(s->suspect_count == original_suspects + 1U);

    velocity = 2;
    memcpy(sample + 4, &velocity, sizeof velocity);
    tick = 2008;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(s->peak_rx.valid && s->peak_rx.timestamp_ms == 2008U);
    assert(memcmp(&s->last_reject, &rejected_speed, sizeof rejected_speed) == 0);

    int16_t position = 2000; /* -72 deg within 2 ms, above jump allowance. */
    memcpy(sample + 2, &position, sizeof position);
    tick = 2010;
    accepted = s->accept_count;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(s->last_reject.reason == MOTOR_TRACE_REASON_POSITION_JUMP);
    assert(fabsf(s->last_reject.position + 72.0f) < 0.001f);
    assert(fabsf(s->last_reject.previous_position + 0.036f) < 0.001f);
    assert(s->last_reject.previous_age_ms == 2U && s->accept_count == accepted);

    position = 1;
    velocity = (int16_t)NAN_INT16;
    memcpy(sample + 2, &position, sizeof position);
    memcpy(sample + 4, &velocity, sizeof velocity);
    tick = 2012;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(s->last_reject.reason == MOTOR_TRACE_REASON_SENTINEL);
    assert(isnan(s->last_reject.velocity) && isfinite(s->last_reject.position));
    assert(memcmp(s->last_reject.raw, sample, sizeof sample) == 0);

    sample[0] = 0x20;
    sample[1] = 0x80;
    tick = 2014;
    motor_process_state(&hfdcan1, 1, sample, sizeof sample);
    assert(s->last_reject.reason == MOTOR_TRACE_REASON_INVALID_TYPE);
    assert(s->last_reject.format == MOTOR_TRACE_FORMAT_UNKNOWN);
    assert(isnan(s->last_reject.position) && isnan(s->last_reject.velocity));

    uint8_t floating[24] = {0x2c, 4, 0, 10};
    const float invalid_position = NAN, finite_velocity = 0.5f, finite_torque = 0.025f;
    memcpy(floating + 7, &invalid_position, sizeof invalid_position);
    memcpy(floating + 11, &finite_velocity, sizeof finite_velocity);
    memcpy(floating + 15, &finite_torque, sizeof finite_torque);
    floating[19] = 0x21;
    floating[20] = 0x0f;
    tick = 2016;
    motor_process_state(&hfdcan1, 1, floating, sizeof floating);
    assert(s->last_reject.reason == MOTOR_TRACE_REASON_NONFINITE);
    assert(s->last_reject.format == MOTOR_TRACE_FORMAT_FULL_FLOAT);
    assert(isnan(s->last_reject.position));
    assert(fabsf(s->last_reject.velocity + 180.0f) < 0.001f);
    assert(s->accept_count == accepted && s->suspect_count == original_suspects + 5U);
    const motor_feedback_trace_t rejected_nan = s->last_reject;

    /* Preserve the original DLC and raw prefix of unknown long frames. Such
     * frames remain ignored and cannot manufacture new suspect events. */
    uint8_t unknown[64];
    memset(unknown, 0x55, sizeof unknown);
    tick = 2018;
    motor_process_state(&hfdcan1, 1, unknown, sizeof unknown);
    assert(s->last_rx.reason == MOTOR_TRACE_REASON_UNRECOGNIZED);
    assert(s->last_rx.data_length == sizeof unknown);
    assert(memcmp(s->last_rx.raw, unknown, sizeof s->last_rx.raw) == 0);
    assert(memcmp(&s->last_reject, &rejected_nan, sizeof rejected_nan) == 0);
    assert(s->suspect_count == original_suspects + 5U && s->accept_count == accepted);
}

static void test_packed_pd_diagnostics(void)
{
    int16_t codes[5] = {0};
    /* Read the SDK's actual packed command; the diagnostic getter sends no CAN
     * frame and must not claim trapezoidal buffers contain PD gains. */
    const unsigned frames_before = control_frames;
    motor_many_pos_vel_acc(PORT1, 1, 10.0f, 20.0f, 40.0f);
    assert(!motor_many_get_pd_codes(PORT1, 1, codes));
    motor_many_pos_vel_tqe_kp_kd_2(PORT1, 1, 50.0f, 0.0f, 0.0f,
                                  8.2f * USB_TWO_PI, 0.5f * USB_TWO_PI);
    assert(motor_many_get_pd_codes(PORT1, 1, codes));
    assert(codes[0] == -1388 && codes[1] == 0 && codes[2] == 0);
    assert(codes[3] == 980 && codes[4] == 59);
    assert(!motor_many_get_pd_codes(PNULL, 1, codes));
    assert(!motor_many_get_pd_codes(PORT3, 1, codes));
    assert(!motor_many_get_pd_codes(PORT1, 0, codes));
    assert(!motor_many_get_pd_codes(PORT1, MOTOR_MAX_NUM + 1, codes));
    assert(!motor_many_get_pd_codes(PORT1, 1, NULL));
    assert(control_frames == frames_before);
}

int main(void)
{
    test_parser();
    test_service();
    test_diagnostic_capture();
    test_packed_pd_diagnostics();
    puts("PASS: actual parser, CAN queries, six-motor enable/stop, feedback and command timeouts");
}
