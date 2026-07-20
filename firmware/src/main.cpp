#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include "conf_hardware.h"
#include "motor_control.h"
#include "encoder.h"
#include "pid.h"

// ------------------------------------------------------------
// micro-ROS objects
// ------------------------------------------------------------
rcl_allocator_t      allocator;
rclc_support_t       support;
rcl_node_t           node;
rcl_subscription_t   cmd_vel_sub;
rcl_publisher_t      odom_pub;
rcl_timer_t          control_timer;
rclc_executor_t      executor;

geometry_msgs__msg__Twist   cmd_vel_msg;
nav_msgs__msg__Odometry     odom_msg;

// ------------------------------------------------------------
// Robot state
// ------------------------------------------------------------
static PidController pid_left;
static PidController pid_right;

static float target_linear  = 0.0f;
static float target_angular = 0.0f;

// ------------------------------------------------------------
// Odometry publisher helpers
// ------------------------------------------------------------
static void publish_odometry(void)
{
    int32_t left_ticks  = 0;
    int32_t right_ticks = 0;
    readEncoders(&left_ticks, &right_ticks);

    float left_speed  = 0.0f;
    float right_speed = 0.0f;
    getWheelSpeeds(left_ticks, right_ticks, &left_speed, &right_speed);

    // Differential drive → body twist
    float linear  = (left_speed + right_speed) * WHEEL_RADIUS / 2.0f;
    float angular = (right_speed - left_speed) * WHEEL_RADIUS / TRACK_WIDTH;

    static uint32_t seq = 0;
    odom_msg.header.stamp.sec      = rmw_uros_epoch_nanos() / 1000000000ULL;
    odom_msg.header.stamp.nanosec  = rmw_uros_epoch_nanos() % 1000000000ULL;
    odom_msg.header.frame_id       = "odom";
    odom_msg.child_frame_id        = "base_footprint";

    odom_msg.twist.twist.linear.x  = linear;
    odom_msg.twist.twist.angular.z = angular;

    // Pose integration (dead-reckoning) — placeholder
    // Full pose update will be added in Phase 1.4 (IMU fusion)
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.header.seq = seq++;

    rcl_publish(&odom_pub, &odom_msg, NULL);
}

// ------------------------------------------------------------
// Control timer callback  (50 ms)
// ------------------------------------------------------------
static void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer != NULL) {
        // 1. Compute target wheel speeds from cmd_vel
        float target_left  = target_linear - target_angular * TRACK_WIDTH / 2.0f;
        float target_right = target_linear + target_angular * TRACK_WIDTH / 2.0f;

        // Normalise to rad/s
        target_left  /= WHEEL_RADIUS;
        target_right /= WHEEL_RADIUS;

        // 2. Read encoders
        int32_t left_ticks  = 0;
        int32_t right_ticks = 0;
        readEncoders(&left_ticks, &right_ticks);

        float left_speed  = 0.0f;
        float right_speed = 0.0f;
        getWheelSpeeds(left_ticks, right_ticks, &left_speed, &right_speed);

        // 3. PID compute
        float dt = 1.0f / CONTROL_LOOP_HZ;
        float left_pwm  = pid_compute(&pid_left,  target_left,  left_speed,  dt);
        float right_pwm = pid_compute(&pid_right, target_right, right_speed, dt);

        // 4. Set motors
        setMotorSpeeds(left_pwm, right_pwm);

        // 5. Publish odometry
        publish_odometry();
    }
}

// ------------------------------------------------------------
// cmd_vel subscription callback
// ------------------------------------------------------------
static void cmdVelCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    target_linear  = msg->linear.x;
    target_angular = msg->angular.z;
}

// ------------------------------------------------------------
// micro-ROS error loop
// ------------------------------------------------------------
static void error_loop(void)
{
    while (1) {
        Serial.println("micro-ROS: transport failure — rebooting in 1 s");
        delay(1000);
        ESP.restart();
    }
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    // Init peripherals
    initEncoders();
    initMotors();

    // Init PID controllers
    PidGains gains = PID_GAINS_DEFAULT;
    pid_init(&pid_left,  gains);
    pid_init(&pid_right, gains);

    // micro-ROS init
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "esp32_motor_controller", "", &support);

    // cmd_vel subscriber
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // odom publisher
    rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");

    // Control timer  (50 ms period)
    rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(CONTROL_LOOP_MS),
        controlCallback);

    // Executor
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &control_timer);

    // Synchronise with agent
    rmw_uros_sync_session(5000);

    Serial.println("micro-ROS node ready — esp32_motor_controller");
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    delay(1);   // yield to watchdog
}
