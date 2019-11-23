package org.firstinspires.ftc.teamcode.Team2844.Drivers;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotHardware
{
    LinearOpMode OpMode_;

    public DcMotor        leftFrontDrive;
    public DcMotor        leftBackDrive;
    public DcMotor        rightFrontDrive;
    public DcMotor        rightBackDrive;
    public DcMotor        lift;
    public Servo          rightGrabber;
    public Servo          leftGrabber;
    public Servo          twistyClaw;
    public DcMotor        rightIntake;
    public DcMotor        leftIntake;
    public DistanceSensor sensorRange;

    public ColorSensor colordriver;
    public DistanceSensor distancedriver;

    BNO055IMU imu;

    private final double     COUNTS_PER_MOTOR_REV       = 28;    //  AndyMark Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION       = 19.2;     // This is < 1.0 if geared UP
    private final double     WHEEL_DIAMETER_INCHES      = 4.0;
    private final double     STRAFING_WHEEL_WIDTH       = 11.0; //FIND
    private final double     LIFT_WHEEL_DIAMETER_INCHES = 2.3;
    private final double     ONE_MOTOR_COUNT            = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION; // 1,120
    final double             COUNTS_PER_INCH            = ONE_MOTOR_COUNT/(WHEEL_DIAMETER_INCHES*3.1416); //TODO determine in class
    final double             COUNTS_PER_INCH_STRAFE     = ONE_MOTOR_COUNT/STRAFING_WHEEL_WIDTH; //FIND
    final double             COUNTS_PER_INCH_LIFT       = ONE_MOTOR_COUNT/(LIFT_WHEEL_DIAMETER_INCHES*3.1416);

    //private final double

    /* Constructor */
    public RobotHardware(HardwareMap ahwMap, LinearOpMode opMode)
    {
        /* Public OpMode members. */
        OpMode_ = opMode;

        colordriver = ahwMap.get(ColorSensor.class, "Color_Sensor");

        distancedriver = ahwMap.get(DistanceSensor.class, "Color_Sensor");

        // Define and Initialize Motors
        rightFrontDrive = ahwMap.get(DcMotor.class, "rfmotor"); // hub 1 motor 0
        rightBackDrive = ahwMap.get(DcMotor.class, "rbmotor"); // hub 1 motor 1
        leftFrontDrive = ahwMap.get(DcMotor.class, "lfmotor"); // hub 1 motor 2
        leftBackDrive = ahwMap.get(DcMotor.class, "lbmotor"); // hub 1 motor 3

        rightGrabber = ahwMap.get(Servo.class, "rgrabber"); // servo 2
        leftGrabber = ahwMap.get(Servo.class, "lgrabber"); // servo 4
        twistyClaw =  ahwMap.get(Servo.class, "twisty"); // idk where

        rightIntake = ahwMap.get(DcMotor.class, "rintake"); // hub 2 motor 0
        leftIntake = ahwMap.get(DcMotor.class, "lintake"); // hub 2 motor 1

        sensorRange = ahwMap.get(DistanceSensor.class, "sensor_range");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // TODO determine which motor should be reversed
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); // TODO determine which motor should be reversed
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// TODO determine which motor should be reversed
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); // TODO determine which motor should be reversed

        leftGrabber.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Set all motors to run without encoders by default
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = ahwMap.get(BNO055IMU.class, "imu");
    }
 }

