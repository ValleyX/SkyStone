package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.LiftEncoderDrive;
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
        LiftEncoderDrive liftEncoderDrive = new LiftEncoderDrive(robot);

        float leftStickY;
        float rightStickY;
        float leftTrigger;
        float rightTrigger;
        float leftTrigger2;
        float rightTrigger2;

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //liftEncoderDrive.MoveToEncoderValue(0.6, 10, 5, true);

        while (opModeIsActive())
        {
            /////////////////////////////GAMEPAD1//////////////////////////////////

            //driving forward/backward
            leftStickY = -gamepad1.left_stick_y; // game pad says up is neg
            telemetry.addData("leftY", leftStickY);
            rightStickY = -gamepad1.right_stick_y; // game pad says up is neg
            telemetry.addData("rightY", rightStickY);
            //telemetry.update();

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;

            if (rightTrigger == 0 && leftTrigger == 0)
            {
                robot.leftFrontDrive.setPower(leftStickY);
                robot.leftBackDrive.setPower(leftStickY);
                robot.rightFrontDrive.setPower(rightStickY);
                robot.rightBackDrive.setPower(rightStickY);
            }

            // foundation grabbers
            if (gamepad1.right_bumper) // raise grabbers
            {
                robot.rightGrabber.setPosition(0.0);
                robot.leftGrabber.setPosition(0.0);
            }
            else if (gamepad1.left_bumper) // lower grabbers
            {
                robot.rightGrabber.setPosition(0.75);
                robot.leftGrabber.setPosition(0.75);


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


            /////////////////////////////GAMEPAD2//////////////////////////////////

            leftTrigger2 = gamepad2.left_trigger;
            rightTrigger2 = gamepad2.right_trigger;

            telemetry.addData("lift encoder value ", liftEncoderDrive.CurrentEncoderPosition());
            telemetry.update();

            // variables
            int ResetLiftHeight = 5;
            int GrabLiftHeight = 10;
            int PlacingLiftHeight = 30;
            double ArmA = 25;
            double ArmB = 75;
            double ArmX = 80;
            double ArmY = 70;
            double arm0 = 0;
            double claw0 = 0; // reset position
            double claw1 = 0.33333; // other position for placing at 90 degrees
            double clawclose = 0;
            double clawopen = 1;
            double platformFlat = 0.75;
            double platformDown = 0;

            // Reset button
            if (gamepad2.right_bumper)
            {
                //liftEncoderDrive.MoveToEncoderValue(0.6, ResetLiftHeight, 5, true);
                robot.swingy.setPosition(arm0);
                robot.twistyClaw.setPosition(claw0);
                //liftEncoderDrive.MoveToEncoderValue(0.6, 0, 5, true);
            }

            // grab block
            if (gamepad2.a)
            {
                //robot.platform.setPosition(platformFlat);
                liftEncoderDrive.MoveToEncoderValue(0.6, GrabLiftHeight, 5, true);
                //robot.clawy.setPosition(clawclose);
                //robot.swingy.setPosition(ArmA);
                //robot.clawy.setPosition(clawopen);
            }

            // placing block @ 0 degrees
            if (gamepad2.b)
            {
                //liftEncoderDrive.MoveToEncoderValue(0.6, PlacingLiftHeight, 5, true);
                robot.swingy.setPosition(ArmB);
                //robot.platform.setPosition(platformDown);
            }

            // placing block at 90 degrees (back)
            if (gamepad2.x)
            {
                //liftEncoderDrive.MoveToEncoderValue(0.6, PlacingLiftHeight, 5, true);
                robot.swingy.setPosition(ArmX);
                robot.twistyClaw.setPosition(claw1);
                //robot.platform.setPosition(platformDown);
            }

            // placing block at 90 degrees (front)
            if (gamepad2.y)
            {
                //liftEncoderDrive.MoveToEncoderValue(0.6, PlacingLiftHeight, 5, true);
                robot.swingy.setPosition(ArmY);
                robot.twistyClaw.setPosition(claw1);
                //robot.platform.setPosition(platformDown);
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

            // lift power on a lever E
            double leftStickY2 = -gamepad2.left_stick_y;
            double currentLiftPosition = liftEncoderDrive.CurrentEncoderPosition();
            double bottomCutOff = 0.1;
            double topCutOff = 33;


            if ((leftStickY2 < 0) && (currentLiftPosition > bottomCutOff))
            {
                 robot.lift.setPower(leftStickY2);
            }
            else if ((leftStickY2 > 0) && (currentLiftPosition < topCutOff))
            {
                robot.lift.setPower(leftStickY2);
            }
            else
            {
                robot.lift.setPower(0);
            }


            if (gamepad1.a)
            {
                robot.swingy.setPosition(0);
                System.out.println("ValleyX gamepad 1 A pressed");
            }
            if (gamepad1.b)
            {
                robot.swingy.setPosition(1);
                System.out.println("ValleyX gamepad 1 B pressed");
            }

            if (gamepad1.x)
            {
                robot.twistyClaw.setPosition(claw0);
            }
            if (gamepad1.y)
            {
                robot.twistyClaw.setPosition(claw1);
            }
        }
    }
}
