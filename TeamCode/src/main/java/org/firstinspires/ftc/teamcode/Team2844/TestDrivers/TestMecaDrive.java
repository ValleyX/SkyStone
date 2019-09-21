package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@TeleOp(name="Test: Test Meca Drive", group="Test")
//@Disabled

public class TestMecaDrive extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);

        final double     COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 40.0;     // This is < 1.0 if geared UP
        final double     ONE_MOTOR_COUNT         = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;

        float leftStickY;
        float rightStickY;
        boolean leftBumper;
        boolean rightBumper;
        float strafePower;
        float leftFrontEncoder;
        float leftBackEncoder;
        float rightFrontEncoder;
        float rightBackEncoder;

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            leftFrontEncoder = robot.leftFrontDrive.getCurrentPosition();
            leftBackEncoder = robot.leftBackDrive.getCurrentPosition();
            rightFrontEncoder = robot.rightFrontDrive.getCurrentPosition();
            rightBackEncoder = robot.rightBackDrive.getCurrentPosition();
            telemetry.addData("encoder leftFront", leftFrontEncoder/ONE_MOTOR_COUNT);
            telemetry.addData("encoder leftBack", leftBackEncoder/ONE_MOTOR_COUNT);
            telemetry.addData("encoder rightFront", rightFrontEncoder/ONE_MOTOR_COUNT);
            telemetry.addData("encoder rightBack", rightBackEncoder/ONE_MOTOR_COUNT);
            //driving forward/backward
            leftStickY = -gamepad1.left_stick_y; // game pad says up is neg
            telemetry.addData("leftY", leftStickY);
            rightStickY = -gamepad1.right_stick_y; // game pad says up is neg
            telemetry.addData("rightY", rightStickY);
            telemetry.update();

            leftBumper = gamepad1.left_bumper;
            rightBumper = gamepad1.right_bumper;
            if (!rightBumper && !leftBumper)
            {
                robot.leftFrontDrive.setPower(leftStickY);
                robot.leftBackDrive.setPower(leftStickY);
                robot.rightFrontDrive.setPower(rightStickY);
                robot.rightBackDrive.setPower(rightStickY);
            }

            if (gamepad1.a)
            {
                robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //strafing right

            strafePower=1;
            if (rightBumper)
            {
                robot.leftFrontDrive.setPower(strafePower);
                robot.leftBackDrive.setPower(-strafePower);
                robot.rightFrontDrive.setPower(-strafePower);
                robot.rightBackDrive.setPower(strafePower);
            }
            else if (!leftBumper)
            {
                robot.leftFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
            }

            //strafing left
            if (leftBumper)
            {
                robot.leftFrontDrive.setPower(-strafePower);
                robot.leftBackDrive.setPower(strafePower);
                robot.rightFrontDrive.setPower(strafePower);
                robot.rightBackDrive.setPower(-strafePower);
            }
            else if (!rightBumper)
            {
                robot.leftFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
            }
        }
    }
}
