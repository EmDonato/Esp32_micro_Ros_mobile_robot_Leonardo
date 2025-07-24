#include "Variable.hpp"

// --- PIN DEFINITIONS ---
const int encoderPinA_L    = 34;  /**< Left encoder channel A pin */
const int encoderPinB_L    = 35;  /**< Left encoder channel B pin */
const int encoderPinA_R    = 16;  /**< Right encoder channel A pin */
const int encoderPinB_R    = 17;  /**< Right encoder channel B pin */

const int pwmPinA    = 32;  /**< PWM enable pin for motor channel A */
const int inPinA     = 33;  /**< Input A for motor driver channel A */
const int inPinB     = 25;  /**< Input B for motor driver channel A */
const int inPinC     = 26;  /**< Input C for motor driver channel B */
const int inPinD     = 27;  /**< Input D for motor driver channel B */
const int pwmPinB    = 14;  /**< PWM enable pin for motor channel B */
const int SDA_PIN    = 21;  /**< I2C SDA pin for IMU */
const int SCL_PIN    = 22;  /**< I2C SCL pin for IMU */

// --- PWM LEDC SETTINGS ---
const int pwmChannel0    = 0;   /**< LEDC channel 0 */
const int pwmChannel1    = 1;   /**< LEDC channel 1 */
const int pwmFreq        = 5000;/**< LEDC PWM frequency in Hz */
const int pwmResolution  = 12;  /**< LEDC PWM resolution in bits */

// --- micro-ROS OBJECTS ---
rcl_publisher_t encoder_pub;              /**< Publisher for odometry messages */
rcl_publisher_t imu_pub;                  /**< Publisher for IMU messages */
rcl_publisher_t debug_pub;               
rcl_subscription_t control_sub;           /**< Subscriber for velocity control */

rcl_node_t enc_node;                      /**< ROS node for encoders */
rcl_node_t ctrl_node;                     /**< ROS node for control */
rcl_node_t imu_node;                     /**< ROS node for imu */
rcl_node_t debug_node;                    


rcl_timer_t timer_odmtry;                 /**< Timer for odometry callbacks */
rcl_timer_t timer_ctrl;                   /**< Timer for control callbacks */
rcl_timer_t timer_IMU;                    /**< Timer for IMU callbacks */
rcl_timer_t timer_enc;                    /**< Timer for encoder callbacks (unused) */

rclc_executor_t executor;                 /**< Executor to handle all ROS tasks */
rclc_support_t support;                   /**< ROS support structure */
rcl_allocator_t allocator;                /**< ROS memory allocator */

int init_uRos_code;                       /**< micro-ROS initialization status code */

// --- ROS MESSAGE BUFFERS ---
float data_array[5];                      /**< Generic data buffer array */
geometry_msgs__msg__Vector3 control_msg;  /**< Received control command */
nav_msgs__msg__Odometry odom_msg;        /**< Odometry message to publish */
geometry_msgs__msg__Twist cmd_vel_msg;    /**< Received control command */
sensor_msgs__msg__Imu imu_msg;            /**< IMU message to publish */
std_msgs__msg__String debug_msg;           /**< debug message to publish */      
// --- WIFI CONFIGURATION ---
char wifiNetName[]  = "FASTWEB-USA6DG";  /**< Wi-Fi network SSID */
char wifiPassword[] = "26KCXAYRSU";      /**< Wi-Fi network password */
char personalIP[]   = "192.168.1.56";    /**< micro-ROS agent IP address */

// --- MOTOR DRIVER INSTANCE ---
L298N motorDriver(pwmChannel0, pwmChannel1, pwmFreq, pwmResolution, 2);

// --- ENCODER INSTANCES AND SYNCHRONIZATION ---
portMUX_TYPE encoderMux_L = portMUX_INITIALIZER_UNLOCKED; /**< Left encoder mutex */
portMUX_TYPE encoderMux_R = portMUX_INITIALIZER_UNLOCKED; /**< Right encoder mutex */
Encoder encoder_L(encoderPinA_L, encoderPinB_L, ENCODER_PPR, 46.8f);
Encoder encoder_R(encoderPinA_R, encoderPinB_R, ENCODER_PPR, 46.8f);

// --- CONTROL PARAMETERS AND PID CONTROLLERS ---
volatile float w_L_ref = 0.0f;            /**< Left wheel angular velocity reference [rad/s] */
volatile float w_R_ref = 0.0f;            /**< Right wheel angular velocity reference [rad/s] */
volatile float rpm_ref_L = 0.0f;          /**< Left wheel RPM reference */
volatile float rpm_ref_R = 0.0f;          /**< Right wheel RPM reference */
volatile float rpm_L = 0.0f;              /**< Measured left wheel RPM */
volatile float rpm_R = 0.0f;              /**< Measured right wheel RPM */
float ki = 25.0f;                         /**< PID I gain */
float kp = 24.0f;                         /**< PID P gain */
float kd = 0.0f;                          /**< PID D gain */
PID pid_L(kp, ki, kd, PID_DT / 1000.0f, INT_MIN_PID, INT_MAX_PID, OUT_MIN, OUT_MAX);
PID pid_R(kp, ki, kd, PID_DT / 1000.0f, INT_MIN_PID, INT_MAX_PID, OUT_MIN, OUT_MAX);

// --- ODOMETRY AND POSE ESTIMATION INSTANCES ---
UnicycleOdometry odometry(WHEEL_RADIUS, WHEEL_SEPARATION);
EncoderPoseEstimator Enc_est(WHEEL_RADIUS, WHEEL_SEPARATION);

// --- IMU STATE ---
volatile float Theta_IMU = 0.0f;          /**< Integrated IMU yaw [rad] */
float gyro_z_bias = 0.0f;                 /**< Gyroscope Z-axis bias [raw units] */
float bias_correction_gain = 100.5f;      /**< Bias correction gain */
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; /**< Mutex for timer ISR */
MPU6050 imu;                             /**< IMU sensor driver instance */

// --- FILTER STATE ---
volatile float theta_filtered = 0.0f;    /**< Filtered heading [rad] */
float alpha_filter_c = 0.5f;              /**< Complementary filter alpha */
float alpha = 0.8f;                       /**< Low-pass filter coefficient */
float dt = DEFAULT_DT;                    /**< Time step for control loop [s] */

