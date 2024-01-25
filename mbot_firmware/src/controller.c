#include "controller.h"
#include <rc/math/filter.h>

const float PWM_MIN = 0.1;
const float PWM_MAX = 0.9;
// const double MAIN_LOOP_PERIOD = 0.1;



int mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg){
    
    left_wheel_pid = rc_filter_empty();
    right_wheel_pid = rc_filter_empty();
    // back_wheel_pid = rc_filter_empty();
    mbot_vx_pid = rc_filter_empty();
    mbot_vy_pid = rc_filter_empty();
    mbot_wz_pid = rc_filter_empty();
    
    // PID initialization
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.left.kp, ctlr_cfg.left.ki, ctlr_cfg.left.kd, ctlr_cfg.left.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_wheel_pid, ctlr_cfg.right.kp, ctlr_cfg.right.ki, ctlr_cfg.right.kd, ctlr_cfg.right.Tf, MAIN_LOOP_PERIOD);
    // rc_filter_pid(&back_wheel_pid, ctlr_cfg.back.kp, ctlr_cfg.back.ki, ctlr_cfg.back.kd, ctlr_cfg.back.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vx_pid, ctlr_cfg.vx.kp, ctlr_cfg.vx.ki, ctlr_cfg.vx.kd, ctlr_cfg.vx.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vy_pid, ctlr_cfg.vy.kp, ctlr_cfg.vy.ki, ctlr_cfg.vy.kd, ctlr_cfg.vy.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_wz_pid, ctlr_cfg.wz.kp, ctlr_cfg.wz.ki, ctlr_cfg.wz.kd, ctlr_cfg.wz.Tf, MAIN_LOOP_PERIOD);
    return 0;

}

int mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, serial_mbot_motor_pwm_t *mbot_motor_pwm,float r,float l){
  
    double right_error = vel_cmd.velocity[0] - vel.velocity[0];
    double left_error = vel_cmd.velocity[1] - vel.velocity[1];
    // double back_error = vel_cmd.velocity[2] - vel.velocity[2];

    // control command using PID
    float right_cmd = rc_filter_march( &right_wheel_pid,right_error);
    float left_cmd = rc_filter_march(&left_wheel_pid,left_error);
    // float back_cmd = rc_filter_march(&back_wheel_pid,back_error);

    // Convert the command to PWM values and scale it
    // right_cmd += r;
    // left_cmd += l;
    // back_cmd += b;

    // // Clamp the PWM to the min and max values
    right_cmd = fmin(fmax(right_cmd+r, PWM_MIN), PWM_MAX);
    left_cmd = fmin(fmax(left_cmd+l, PWM_MIN), PWM_MAX);
    // back_cmd = fmin(fmax(back_cmd+b, PWM_MIN), PWM_MAX);

    // convert the PWN value to command
    mbot_motor_pwm->pwm[0] = right_cmd;
    mbot_motor_pwm->pwm[1] = left_cmd;
    // mbot_motor_pwm->pwm[2] = back_cmd;

    return 0;
}
