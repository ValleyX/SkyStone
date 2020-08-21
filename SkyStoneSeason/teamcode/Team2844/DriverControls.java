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

        float leftStickY;
        float rightStickY;
        float leftTrigger;
        float rightTrigger;
        float leftTrigger2;
        float rightTrigger2;

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            //driving forward/backward
            leftStickY = -gamepad1.left_stick_y; // game pad says up is neg
            telemetry.addData("leftY", leftStickY);
            rightStickY = -gamepad1.right_stick_y; // game pad says up is neg
            telemetry.addData("rightY", rightStickY);
            telemetry.update();

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;

            leftTrigger2 = gamepad2.left_trigger;
            rightTrigger2 = gamepad2.right_trigger;

            if (rightTrigger == 0 && leftTrigger == 0)
            {
                robot.leftFrontDrive.setPower(leftStickY);
                robot.leftBackDrive.setPower(leftStickY);
                robot.rightFrontDrive.setPower(rightStickY);
                robot.rightBackDrive.setPower(rightStickY);
            }

            //strafing right
            if (rightTrigger > 0)
            {
                robot.leftFrontDrive.setPower(-rightTrigger); // +
                robot.leftBackDrive.setPower(rightTrigger); // -
                robot.rightFrontDrive.setPower(rightTrigger); // -
                robot.rightBackDrive.setPower(-rightTrigger); // +
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
                robot.leftFrontDrive.setPower(leftTrigger); // -
                robot.leftBackDrive.setPower(-leftTrigger); // +
                robot.rightFrontDrive.setPower(-leftTrigger); // +
                robot.rightBackDrive.setPower(leftTrigger); // -
            }
            else if (rightTrigger == 0)
            {
                robot.leftFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
            }

            // foundation grabbers
            if (gamepad2.x) // raise grabbers
            {
                robot.rightGrabber.setPosition(0.0);
                robot.leftGrabber.setPosition(0.0);
            }
            else if (gamepad2.y) // lower grabbers
            {
                robot.rightGrabber.setPosition(0.75);
                robot.leftGrabber.setPosition(0.75);
            }

            // intake
            if (rightTrigger2 > 0)
            {
                robot.rightIntake.setPower(rightTrigger2);
                robot.leftIntake.setPower(-rightTrigger2);
            }
            else if (leftTrigger2 > 0)
            {
                robot.rightIntake.setPower(-leftTrigger2);
                robot.leftIntake.setPower(leftTrigger2);
            }
            else
            {
                robot.rightIntake.setPower(0);
                robot.leftIntake.setPower(0);
            }
            idle();
        }
    }
}
