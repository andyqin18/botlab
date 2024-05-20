/**
 * This file is the main executable for the MBot firmware.
 */
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include "mbot.h"
#include "odometry.h"
#include <mbot/utils/utils.h>
#include "print_tables.h"
#include <mbot/defs/mbot_params.h>

#include "controller.h"
#include <rc/math/filter.h>

#define THETA "\u0398"
#pragma pack(1)

#ifndef MBOT_DRIVE_TYPE
#error "Please define a drive type for the bot"
#endif

#if MBOT_DRIVE_TYPE != DIFFERENTIAL_DRIVE
#error "This file has been modified to work only with differential drive."
#endif

// Global
static uint64_t timestamp_offset = 0;
static uint64_t global_start_utime = 0;
static uint64_t global_utime = 0;
static uint64_t global_pico_time = 0;
static bool global_comms_status = COMMS_ERROR;
static int drive_mode = MODE_MBOT_VEL;
static bool running = false;
static mbot_params_t params;

mbot_bhy_data_t mbot_imu_data;

void print_mbot_params(const mbot_params_t* params) {
    printf("Robot Type: %d\n", params->robot_type);
    printf("Wheel Radius: %f\n", params->wheel_radius);
    printf("Wheel Base Radius: %f\n", params->wheel_base_radius);
    printf("Gear Ratio: %f\n", params->gear_ratio);
    printf("Encoder Resolution: %f\n", params->encoder_resolution);
    printf("Motor Left: %d\n", params->mot_left);
    printf("Motor Right: %d\n", params->mot_right);
    printf("Motor Back: %d\n", params->mot_back);
    printf("Motor Polarity: %d %d %d\n", params->motor_polarity[0], params->motor_polarity[1], params->motor_polarity[2]);
    printf("Encoder Polarity: %d %d %d\n", params->encoder_polarity[0], params->encoder_polarity[1], params->encoder_polarity[2]);
    printf("Positive Slope: %f %f %f\n", params->slope_pos[0], params->slope_pos[1], params->slope_pos[2]);
    printf("Positive Intercept: %f %f %f\n", params->itrcpt_pos[0], params->itrcpt_pos[1], params->itrcpt_pos[2]);
    printf("Negative Slope: %f %f %f\n", params->slope_neg[0], params->slope_neg[1], params->slope_neg[2]);
    printf("Negative Intercept: %f %f %f\n", params->itrcpt_neg[0], params->itrcpt_neg[1], params->itrcpt_neg[2]);
}

void register_topics()
{
    // Subscriptions
    comms_register_topic(MBOT_TIMESYNC, sizeof(serial_timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    comms_register_topic(MBOT_ODOMETRY_RESET,  sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, (MsgCb)&reset_odometry_cb);
    comms_register_topic(MBOT_ENCODERS_RESET, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, (MsgCb)&reset_encoders_cb);
    comms_register_topic(MBOT_MOTOR_PWM_CMD, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, (MsgCb)mbot_motor_vel_cmd_cb);
    comms_register_topic(MBOT_MOTOR_VEL_CMD, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, (MsgCb)mbot_motor_pwm_cmd_cb);
    comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, (MsgCb)mbot_vel_cmd_cb);

    // Published Topics
    comms_register_topic(MBOT_ODOMETRY, sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, NULL);
    comms_register_topic(MBOT_IMU, sizeof(serial_mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    comms_register_topic(MBOT_ENCODERS, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, NULL);
    comms_register_topic(MBOT_VEL, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_VEL, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_PWM, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, NULL);

}

void timestamp_cb(serial_timestamp_t *msg)
{
    global_pico_time = to_us_since_boot(get_absolute_time());
    timestamp_offset = msg->utime - global_pico_time;
    global_comms_status = COMMS_OK;
}

void reset_encoders_cb(serial_mbot_encoders_t *msg)
{
    //memcpy(&encoders, msg, sizeof(serial_mbot_encoders_t));
    for(int i=0; i<3; i++){
        mbot_encoder_write(i, msg->ticks[i]);
    }
}

void reset_odometry_cb(serial_pose2D_t *msg)
{
    mbot_odometry.x = msg->x;
    mbot_odometry.y = msg->y;
    mbot_odometry.theta = msg->theta;
}

void mbot_vel_cmd_cb(serial_twist2D_t *msg)
{
    memcpy(&mbot_vel_cmd, msg, sizeof(serial_twist2D_t));
    drive_mode = MODE_MBOT_VEL;
}

void mbot_motor_vel_cmd_cb(serial_mbot_motor_vel_t *msg)
{
    memcpy(&mbot_motor_vel_cmd, msg, sizeof(serial_mbot_motor_vel_t));
    drive_mode = MODE_MOTOR_VEL_OL;
}

void mbot_motor_pwm_cmd_cb(serial_mbot_motor_pwm_t *msg)
{
    memcpy(&mbot_motor_pwm_cmd, msg, sizeof(serial_mbot_motor_pwm_t));
    drive_mode = MODE_MOTOR_PWM;
}

void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel){
    float conversion = (1.0 / params.gear_ratio) * (1.0 / params.encoder_resolution) * 1E6f * 2.0 * M_PI;
    motor_vel->velocity[params.mot_left] = params.encoder_polarity[params.mot_left] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_left];
    motor_vel->velocity[params.mot_right] = params.encoder_polarity[params.mot_right] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_right];
}

void mbot_read_imu(serial_mbot_imu_t *imu){
    imu->utime = global_utime;
    imu->gyro[0] = mbot_imu_data.gyro[0];
    imu->gyro[1] = mbot_imu_data.gyro[1];
    imu->gyro[2] = mbot_imu_data.gyro[2];
    imu->accel[0] = mbot_imu_data.accel[0];
    imu->accel[1] = mbot_imu_data.accel[1];
    imu->accel[2] = mbot_imu_data.accel[2];
    imu->mag[0] = mbot_imu_data.mag[0];
    imu->mag[1] = mbot_imu_data.mag[1];
    imu->mag[2] = mbot_imu_data.mag[2];
    imu->angles_rpy[0] = mbot_imu_data.rpy[0];
    imu->angles_rpy[1] = mbot_imu_data.rpy[1];
    imu->angles_rpy[2] = mbot_imu_data.rpy[2];
    imu->angles_quat[0] = mbot_imu_data.quat[0];
    imu->angles_quat[1] = mbot_imu_data.quat[1];
    imu->angles_quat[2] = mbot_imu_data.quat[2];
    imu->angles_quat[3] = mbot_imu_data.quat[3];
}

void mbot_read_encoders(serial_mbot_encoders_t* encoders){
    int64_t delta_time = global_utime - encoders->utime;
    encoders->utime = global_utime;
    encoders->delta_time = delta_time;

    encoders->ticks[params.mot_right] = mbot_encoder_read_count(params.mot_right);
    encoders->delta_ticks[params.mot_right] = mbot_encoder_read_delta(params.mot_right);
    encoders->ticks[params.mot_left] = mbot_encoder_read_count(params.mot_left);
    encoders->delta_ticks[params.mot_left] = mbot_encoder_read_delta(params.mot_left);
}

int mbot_init_pico(void){
    bi_decl(bi_program_description("Firmware for the MBot Robot Control Board"));

    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
    if(!set_sys_clock_khz(125000, true)){
        printf("ERROR mbot_init_pico: cannot set system clock\n");
        return MBOT_ERROR;
    };

    stdio_init_all(); // enable USB serial terminal
    sleep_ms(500);
    printf("\nMBot Booting Up!\n");
    return MBOT_OK;
}

int mbot_init_hardware(void){
    sleep_ms(1000);
    // Initialize Motors
    printf("initializinging motors...\n");
    mbot_motor_init(DIFF_MOTOR_LEFT_SLOT);
    mbot_motor_init(DIFF_MOTOR_RIGHT_SLOT);
    printf("initializinging encoders...\n");
    mbot_encoder_init();

    // Initialize LED
    printf("Starting heartbeat LED...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    mbot_bhy_config_t mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.sample_rate = 200;
    // Initialize the IMU using the Digital Motion Processor
    printf("Initializing IMU...\n");
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);
    mbot_init_fram();
    return MBOT_OK;
}

int mbot_init_comms(void){
    printf("Initializing LCM serial communication...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
    return MBOT_OK;
}

int mbot_init_pid() {
    // PID constants, assuming the parameters for the left and right wheels
    // should be the same.
    const float kp = 1.6f;
    const float ki = 0.8f;
    const float kd = 0.01f;
    const float Tf = MAIN_LOOP_PERIOD * 4;

    const mbot_pid_cfg_t left_wheel_pid_params = {.kp = kp, .ki = ki, .kd = kd, .Tf = Tf};
    const mbot_pid_cfg_t right_wheel_pid_params = {.kp = kp, .ki = ki, .kd = kd, .Tf = Tf};

    mbot_ctlr_cfg_t ctlr_cfg;
    ctlr_cfg.left = left_wheel_pid_params;
    ctlr_cfg.right = right_wheel_pid_params;

    // We're not using these PIDs yet, but we need to set Tf to silence errors.
    ctlr_cfg.vx.Tf = Tf;
    ctlr_cfg.vy.Tf = Tf;
    ctlr_cfg.wz.Tf = Tf;

    mbot_init_ctlr(ctlr_cfg);

    return MBOT_OK;
}

void mbot_print_state(serial_mbot_imu_t imu, serial_mbot_encoders_t encoders, serial_pose2D_t odometry, serial_mbot_motor_vel_t motor_vel){
    printf("\033[2J\r");
    if(global_comms_status == COMMS_OK){
        const float relative_time = (global_utime - global_start_utime) * 1e-6;
        printf("| \033[32m COMMS OK \033[0m TIME: %16.3f |\n", relative_time);
    }
    else{
        printf("| \033[31m SERIAL COMMUNICATION FAILURE\033[0m     |\n");
    }
    const char* imu_headings[] = {"ROLL", "PITCH", "YAW"};
    const char* enc_headings[] = {"ENC 0", "ENC 1", "ENC 2"};
    const char* odom_headings[] = {"X", "Y", "THETA"};
    const char* motor_vel_headings[] = {"MOT 0", "MOT 1", "MOT 2"};
    // we shouldn't need to do this, need to update generateTable to handle different datatypes
    int encs[3] = {(int)encoders.ticks[0], (int)encoders.ticks[1], (int)encoders.ticks[2]};
    char buf[1024] = {0};
    generateTableInt(buf, 1, 3, "ENCODERS", enc_headings, encs);
    printf("\r%s", buf);

    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "IMU POS (rad)", imu_headings, imu.angles_rpy);
    printf("\r%s", buf);

    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "MOTOR VEL (rad/s)", motor_vel_headings, motor_vel.velocity);
    printf("\r%s", buf);

    buf[0] = '\0';
    float odom_array[3] = {odometry.x, odometry.y, odometry.theta};
    generateTableFloat(buf, 1, 3, "ODOMETRY", odom_headings, odom_array);
    printf("\r%s", buf);

    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "VEL FEEDBACK", odom_headings, &mbot_vel.vx);
    printf("\r%s", buf);

    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "VEL CMD", odom_headings, &mbot_vel_cmd.vx);
    printf("\r%s", buf);

    buf[0] = '\0';
    generateBottomLine(buf, 3);
    printf("\r%s\n", buf);
}

// Simplified version of the mbot_print_state_linearized function that prints
// state information in csv format without clearing the screen each call. This
// function is useful for downstream time-value graphs.
void mbot_print_state_csv(const serial_mbot_imu_t imu,
                                 const serial_mbot_encoders_t encoders,
                                 const serial_pose2D_t odometry,
                                 const serial_mbot_motor_vel_t motor_vel) {
  const float relative_time = (global_utime - global_start_utime) * 1e-6;
  printf("%.3f,", relative_time);
  printf("%.3f,%.3f,%.3f ", odometry.x, odometry.y, odometry.theta);
  printf("%.3f,%.3f,%.3f", mbot_vel.vx, mbot_vel.vy, mbot_vel.wz);
  printf("\n");
}

//Helper function to use slope + intercept from calibration to generate a PWM command.
float _calibrated_pwm_from_vel_cmd(const float vel_cmd, const int motor_idx){
    if (vel_cmd > 0.0)
    {
        return (vel_cmd * params.slope_pos[motor_idx]) + params.itrcpt_pos[motor_idx];
    }
    else if (vel_cmd < 0.0)
    {
        return (vel_cmd * params.slope_neg[motor_idx]) + params.itrcpt_neg[motor_idx];
    }
    return 0.0;
}

// Helper function to generate a PWM command based on a commanded velocity and
// the encoder feedback using PID.
float _calibrated_pwm_from_vel_cmd_feedback(const float vel_cmd,
                                            const int motor_idx,
                                            const serial_mbot_motor_vel_t* motor_vel) {
  const float error = motor_vel->velocity[motor_idx] - vel_cmd;
  float new_vel_cmd = 0.0;

  if (fabsf(vel_cmd) < 0.1) {
    // Enforce a minimum threshold so the motor does not oscillate about zero.
    new_vel_cmd = 0.0f;
  } else if (motor_idx == params.mot_left) {
    new_vel_cmd = rc_filter_march(&left_wheel_pid, error);
  } else if (motor_idx == params.mot_right) {
    new_vel_cmd = rc_filter_march(&right_wheel_pid, error);
  }

  return _calibrated_pwm_from_vel_cmd(new_vel_cmd, motor_idx);

  // Debug code:
  //   const float ret = _calibrated_pwm_from_vel_cmd(new_vel_cmd, motor_idx);
  //   if (motor_idx == 0) {
  //     printf(
  //         "_calibrated_pwm_from_vel_cmd: idx: %d, fdbk: %.3f, new cmd: %.3f, old cmd: %.3f, err:
  //         %.3f, pwm: %.3f, " "target: %.3f\n", motor_idx, motor_vel->velocity[motor_idx],
  //         new_vel_cmd, vel_cmd, error, ret, _calibrated_pwm_from_vel_cmd(vel_cmd, motor_idx));
  //   }
  //   return ret;
}

// TODO: this could be tied to the IMU interrupt
bool mbot_loop(repeating_timer_t *rt)
{
    //
    // IMPORTANT
    //
    // For 550 we will assume that the drive type is differential drive to
    // simplify the logic in this function.
    //
    global_utime = to_us_since_boot(get_absolute_time()) + timestamp_offset;
    mbot_vel.utime = global_utime;
    mbot_read_encoders(&mbot_encoders);
    mbot_read_imu(&mbot_imu);
    mbot_calculate_motor_vel(mbot_encoders, &mbot_motor_vel);

    mbot_calculate_diff_body_vel(mbot_motor_vel.velocity[params.mot_left],
                                 mbot_motor_vel.velocity[params.mot_right], &mbot_vel);
    mbot_calculate_odometry(mbot_vel, mbot_imu, MAIN_LOOP_PERIOD, &mbot_odometry);
    mbot_odometry.utime = global_utime;
    // only run if we've got 2 way communication...
    if (global_comms_status == COMMS_OK)
    {
        if(drive_mode == MODE_MOTOR_VEL_OL){
            mbot_motor_pwm.utime = global_utime;
            mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd_feedback(mbot_motor_vel_cmd.velocity[params.mot_right], params.mot_right, &mbot_motor_vel);
            mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd_feedback(mbot_motor_vel_cmd.velocity[params.mot_left], params.mot_left, &mbot_motor_vel);
        }else if(drive_mode == MODE_MBOT_VEL){
            //TODO: open loop for now - implement closed loop controller

            //
            // IMPORTANT
            //
            // The left and right assignment below is swapped compared to the
            // original code. We had to make this change in order to move
            // forward with a positive velocity command on Elvin's robot,
            // although we are not sure why it was necessary in the first place.
            //
            mbot_motor_vel_cmd.velocity[params.mot_left] = -(mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
            mbot_motor_vel_cmd.velocity[params.mot_right] = -(-mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;

            const float vel_left_comp = params.motor_polarity[params.mot_left] * mbot_motor_vel_cmd.velocity[params.mot_left];
            const float vel_right_comp = params.motor_polarity[params.mot_right] * mbot_motor_vel_cmd.velocity[params.mot_right];

            mbot_motor_pwm.utime = global_utime;
            mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd_feedback(vel_right_comp, params.mot_right, &mbot_motor_vel);
            mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd_feedback(vel_left_comp, params.mot_left, &mbot_motor_vel);
        }else {
            drive_mode = MODE_MOTOR_PWM;
            mbot_motor_pwm.utime = global_utime;
        }

        // Set motors
        mbot_motor_set_duty(params.mot_right, mbot_motor_pwm_cmd.pwm[params.mot_right]);
        mbot_motor_pwm.pwm[params.mot_right] = mbot_motor_pwm_cmd.pwm[params.mot_right];
        mbot_motor_set_duty(params.mot_left, mbot_motor_pwm_cmd.pwm[params.mot_left]);
        mbot_motor_pwm.pwm[params.mot_left] = mbot_motor_pwm_cmd.pwm[params.mot_left];

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &mbot_encoders);
        // send odom on wire
        comms_write_topic(MBOT_ODOMETRY, &mbot_odometry);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &mbot_imu);
        // write the Body velocity to serial
        comms_write_topic(MBOT_VEL, &mbot_vel);
        // write the Motor velocity to serial
        comms_write_topic(MBOT_MOTOR_VEL, &mbot_motor_vel);
        // write the PWMs to serial
        comms_write_topic(MBOT_MOTOR_PWM, &mbot_motor_pwm);
        //uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }
    //check comms and kill motors if its been too long
    uint64_t timeout = to_us_since_boot(get_absolute_time()) - global_pico_time;
    if(timeout > MBOT_TIMEOUT_US){
        mbot_motor_set_duty(DIFF_MOTOR_LEFT_SLOT, 0.0);
        mbot_motor_set_duty(DIFF_MOTOR_RIGHT_SLOT, 0.0);
        global_comms_status = COMMS_ERROR;
    }

    return true;
}


int main()
{
    running = false;
    mbot_init_pico();
    mbot_init_hardware();
    mbot_init_comms();
    mbot_read_fram(0, sizeof(params), (uint8_t*)(&params));
    mbot_init_pid();

    //Check also that define drive type is same as FRAM drive type
    int validate_status = validate_FRAM_data(&params);
    if (validate_status < 0)
    {
        printf("Failed to validate FRAM Data! Error code: %d\n", validate_status);
        return -1;
    }

    if(params.robot_type != MBOT_DRIVE_TYPE){
        printf("#define type is not equal to calibration type!\n");
        return -1;
    }

    sleep_ms(3000);
    print_mbot_params(&params);
    printf("Starting MBot Loop...\n");
    global_start_utime = to_us_since_boot(get_absolute_time()) + timestamp_offset;
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, mbot_loop, NULL, &loop_timer); // 1000x to convert to ms
    printf("Done Booting Up!\n");
    running = true;
    uint16_t counter = 0;

    while(running){
        // Heartbeat
        if(!(counter % 5)){
            gpio_put(LED_PIN, 1);
        }
        else if(!(counter % 7)){
            gpio_put(LED_PIN, 1);
            counter = 0;
        }
        else{
            gpio_put(LED_PIN, 0);
        }
        // Print State
        mbot_print_state(mbot_imu, mbot_encoders, mbot_odometry, mbot_motor_vel);
        sleep_ms(200);
        counter++;
    }
}
