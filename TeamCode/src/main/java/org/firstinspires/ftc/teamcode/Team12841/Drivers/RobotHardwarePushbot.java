package org.firstinspires.ftc.teamcode.Team12841.Drivers;

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 *
 *
 */
public class RobotHardwarePushbot
{
    LinearOpMode OpMode_;
/*
    public DcMotor  LeftFrontDrive;
    public DcMotor  RightFrontDrive;
    public DcMotor  LeftBackDrive;
    public DcMotor  RightBackDrive;
    */
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public Servo  BackLatch;


    private final double     COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 40.0;     // This is < 1.0 if geared UP
    private final double     ONE_MOTOR_COUNT         = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private final double     Diameter                = 4;
    private final double     Pie                     = 3.141592;
    final double             COUNTS_PER_INCH         = ONE_MOTOR_COUNT/(Diameter * Pie);  //TODO determine in class

    /* Constructor */
    public RobotHardwarePushbot(HardwareMap ahwMap, LinearOpMode opMode) {
        /* Public OpMode members. */
        OpMode_ = opMode;

        sensorColor = ahwMap.get(ColorSensor.class, "sensor_color_distance");

        sensorDistance = ahwMap.get(DistanceSensor.class, "sensor_color_distance");

        BackLatch = ahwMap.get(Servo.class, "BackLatch"); // Servo 0

        BackLatch.setDirection(Servo.Direction.FORWARD);

        // Set all motors to zero power

        BackLatch.setPosition(0);

        // Set all motors to run without encoders by default


    }
 }

