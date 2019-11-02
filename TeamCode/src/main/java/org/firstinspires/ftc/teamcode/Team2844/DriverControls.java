package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

//import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp: Driver Controls", group="Test")
//@Disabled

public class DriverControls extends LinearOpMode
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
        float leftTrigger;
        float rightTrigger;
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

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (rightTrigger == 0 && leftTrigger == 0)
            {
                robot.leftFrontDrive.setPower(leftStickY);
                robot.leftBackDrive.setPower(leftStickY);
                robot.rightFrontDrive.setPower(rightStickY);
                robot.rightBackDrive.setPower(rightStickY);
            }

            //strafing right
            strafePower=1;
            if (rightTrigger > 0)
            {
                robot.leftFrontDrive.setPower(rightTrigger);
                robot.leftBackDrive.setPower(-rightTrigger);
                robot.rightFrontDrive.setPower(-rightTrigger);
                robot.rightBackDrive.setPower(rightTrigger);
            }
            else if (leftTrigger == 0)
            {
                robot.leftFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
            }

            //strafing left
            if (leftTrigger > 0)
            {
                robot.leftFrontDrive.setPower(-leftTrigger);
                robot.leftBackDrive.setPower(leftTrigger);
                robot.rightFrontDrive.setPower(leftTrigger);
                robot.rightBackDrive.setPower(-leftTrigger);
            }
            else if (rightTrigger == 0)
            {
                robot.leftFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
            }
        }
    }
}
