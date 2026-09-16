/*
 /\_/\  /\_/\  /\_/\  /\_/\        VSAR_26.ino:
( o.o )( o.o )( o.o )( o.o )       |___CONFIGURATION: Constants and channels
 > ^ <  > ^ <  > ^ <  > ^ <        |___PWM DRIVER: Adafruit PWM Servo Driver
#######              #######       |___HARDWARE API: DCMotor
 /\_/\    ghelopax    /\_/\        |___DRIVETRAIN: Mecanum Drive (PS2 Control)
( o.o )    narra     ( o.o )       |___ARDUINO FUNCTIONS: setup(), loop()
 > ^ <   @itsmevjnk   > ^ <
#######              #######
 /\_/\  /\_/\  /\_/\  /\_/\
( o.o )( o.o )( o.o )( o.o )
 > ^ <  > ^ <  > ^ <  > ^ <
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define SPC Serial.print(" ")

#define RUN

// #############
// CONFIGURATION
// #############

/* CONSTANTS */
// Motor speed
#define SPD_MAX 4095
#define SPD_DEAD 70
#define PER(percentage) (int16_t)(SPD_MAX * (percentage))

#define SPD_DRIVE_LF PER(0.67)
#define SPD_DRIVE_LB PER(0.67)
#define SPD_DRIVE_RF PER(0.67)
#define SPD_DRIVE_RB PER(0.67)

// Tốc độ Intake JGB37-555
#define SPD_INTAKE PER(0.50)

/* PWM channels */
// DC Motor Drivetrain
#define LF_A 6 // Mecanum Drive
#define LF_B 7
#define LB_A 4
#define LB_B 5
#define RF_A 2
#define RF_B 3
#define RB_A 0
#define RB_B 1

// Intake Channel (Kênh 10 và 11 trên PCA9685)
#define INTAKE_A 10
#define INTAKE_B 11

/* PS2 pins */
#define PS2_DAT 13
#define PS2_CMD 11
#define PS2_ATT 10
#define PS2_CLK 12
// outtake
#define OUTTAKE_A 14
//dieu chinh goc do servo
#define SERVO_CLOSED_US 500
#define SERVO_OPEN_US 1500
// ##########
// PWM DRIVER
// ##########

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void init_PWMDriver()
{
    Serial.print(F("Initializing PCA9685..."));

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    Wire.setClock(400000);

    Serial.println(F("done."));
}

// ############
// HARDWARE API
// ############

/* PS2 Controller */
PS2X ps2;

void init_PS2()
{
    Serial.print(F("Initializing PS2 controller..."));

    uint8_t error = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
    while (error != 0)
    {
        switch (error)
        {
        case 1:
            Serial.println("\nError code 1: No controller found, check wiring.");
            break;
        case 2:
            Serial.println("\nError code 2: Controller found but not accepting commands.");
            break;
        case 3:
            Serial.println("\nError code 3: Controller refusing to enter Pressures mode, may not support it.");
            break;
        }
        delay(1000);
        error = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
    }

    Serial.println(F("done."));
}

/* Control */
struct DCMotor
{
private:
    uint8_t channelA, channelB;
    bool reverse;

public:
    DCMotor(uint8_t _channelA, uint8_t _channelB, bool _reverse = false) : channelA(_channelA),
                                                                           channelB(_channelB),
                                                                           reverse(_reverse)
    {
    }

    void control(int16_t speed)
    {
        if (abs(speed) < SPD_DEAD)
            speed = 0;
        if (reverse)
            speed = -speed;

#ifdef RUN
        pwm.setPWM(channelA, 0, ((speed > 0) ? speed : 0));
        pwm.setPWM(channelB, 0, ((speed < 0) ? (-speed) : 0));
#endif
    }

    void relControl(float rel_speed)
    {
        control(PER(rel_speed));
    }
};

DCMotor intake(INTAKE_A, INTAKE_B);

// #########################
// DRIVETRAIN: Mecanum Drive
// #########################

struct Drivetrain
{
private:
    DCMotor leftfront, leftback, rightfront, rightback;

public:
    Drivetrain() : leftfront(LF_A, LF_B),
                   leftback(LB_A, LB_B),
                   rightfront(RF_A, RF_B, true),
                   rightback(RB_A, RB_B, true)
    {
    }

    void update(uint8_t stra, uint8_t forw, uint8_t rota)
    {
        int16_t x = map(stra, 0, 255, -SPD_MAX, SPD_MAX);
        int16_t y = map(forw, 0, 255, SPD_MAX, -SPD_MAX);
        int16_t r = map(rota, 0, 255, SPD_MAX, -SPD_MAX);
        int16_t d = max(abs(x) + abs(y) + abs(r), SPD_MAX);

        int16_t lf = (int32_t)(x + y - r) * SPD_DRIVE_LF / d;
        int16_t lb = (int32_t)(-x + y - r) * SPD_DRIVE_LB / d;
        int16_t rf = (int32_t)(-x + y + r) * SPD_DRIVE_RF / d;
        int16_t rb = (int32_t)(x + y + r) * SPD_DRIVE_RB / d;

        leftfront.control(lf);
        leftback.control(lb);
        rightfront.control(rf);
        rightback.control(rb);
    }
} drivetrain;

// #################
// ARDUINO FUNCTIONS
// #################

bool intakeRunning = false;
bool outtakeOpen = false;
void setup()
{
    Serial.begin(115200);
    Serial.println("Robots In A Nutshell\nVSAR 2026 : RIAN C\n[MECANUM + INTAKE DEBUG]");

    init_PWMDriver();
    init_PS2();
    pwm.writeMicroseconds(OUTTAKE_A, SERVO_CLOSED_US);
}

void loop()
{
    ps2.read_gamepad(); // Đọc tín hiệu từ tay cầm PS2

    // 1. Cập nhật di chuyển Drivetrain
    drivetrain.update(
        ps2.Analog(PSS_LX),
        ps2.Analog(PSS_LY),
        ps2.Analog(PSS_RX));

    // 2. Điều khiển động cơ Intake
    if (ps2.ButtonPressed(PSB_L1))
    {
        intakeRunning = !intakeRunning;
        if (intakeRunning)
        {
            intake.control(-SPD_INTAKE); // Bật quay ngược
        }
        else
        {
            intake.control(0); // Tắt
        }
    }
    // 3. NHẬN NÚT R1 ĐỂ BẬT/TẮT OUTTAKE
    if (ps2.ButtonPressed(PSB_R1))
    {
        outtakeOpen = !outtakeOpen;

        if (outtakeOpen)
        {
            pwm.writeMicroseconds(OUTTAKE_A, SERVO_OPEN_US);
        }
        else
        {
            pwm.writeMicroseconds(OUTTAKE_A, SERVO_CLOSED_US);
        }
    }
    delay(20);
}
