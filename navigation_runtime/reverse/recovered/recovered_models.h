#ifndef FEEDBOT_RECOVERED_MODELS_H
#define FEEDBOT_RECOVERED_MODELS_H
/* Offline semantic models: no device access. See each implementation's scope. */
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { uint32_t function, a, b, c; } Call;
typedef struct {
 uint32_t tick, last_tick;
 uint16_t silence_count, pending_action;
 uint8_t flag_c2, flag_c1;
 uint32_t call_count;
 Call calls[16];
} State;
typedef struct {uint8_t due,pad;uint16_t remaining,period,pad2;uint32_t function;} Task;
void recovered_periodic(State *state);
int recovered_check_frame(const uint8_t *data, size_t length);
void recovered_scheduler(Task tasks[8]);
int recovered_found_charge(unsigned char *state, unsigned char *twist, int perception_result, uint64_t now_bits);
void recovered_charge_state(unsigned char *state, uint8_t value);
void recovered_charge_approach_slice(unsigned char *state, unsigned char *twist, double error, int second_entry);
void recovered_charge_final_slice(unsigned char *state, unsigned char *twist, double error, double distance, int reverse);
int recovered_approach_planar(unsigned char *state, unsigned char *twist, const double *robot, const double *dock);
int recovered_final_planar(unsigned char *state, unsigned char *twist, const double *robot, const double *dock);
uint32_t recovered_smooth_pwm(float state[2], int16_t v, int16_t w,
    int32_t sec, int32_t nsec, float cap, float range, const double params[4]);
int recovered_driver_frame(uint8_t *state, int message, int16_t smoothed_linear,
    uint8_t packet[40]);
float recovered_dwa_pid(unsigned char *object, float command, int direction);
uint16_t recovered_lidar_crc(const uint8_t *data, size_t length);
int recovered_lidar_verify(uint8_t *data, int length);
typedef int (*charge_effect)(int op, int arg, void *context);
int recovered_charge_execute(unsigned char *state, unsigned char *twist,
    charge_effect effect, void *context);
typedef struct {
    int16_t previous_v,previous_w,target_v,target_w;
    uint32_t calls,argument;
} target_state;
void recovered_sub_target(target_state *state, float sensor, int mode,
    int32_t v, int16_t w, uint32_t arg);
void recovered_sub_kinematics(unsigned char *state, uint8_t mode, int gate,
    const uint16_t near[3], uint32_t arg, int32_t out[3]);
typedef struct {uint32_t id; uint8_t data[8];} MotorFrame;
typedef struct {uint32_t count;MotorFrame frames[2];} MotorOutput;
void recovered_motor_frame(uint32_t node, int32_t target, MotorFrame *out);
void recovered_motor_commit(uint8_t state[0x48], uint8_t flags[2],
    int32_t target_a, int32_t target_b, uint32_t span, MotorOutput *out);
#ifdef __cplusplus
}
#endif
#endif
