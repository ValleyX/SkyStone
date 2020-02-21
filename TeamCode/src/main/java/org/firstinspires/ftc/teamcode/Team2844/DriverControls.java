package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.FlippyDriver;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.LiftEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.flippyEncoderDrive;


//import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp: Driver Controls", group="Test")
//@Disabled

public class DriverControls extends LinearOpMode
{
    RobotHardware robot;
    private ElapsedTime runtime_;
    LiftEncoderDrive liftEncoderDrive;
    flippyEncoderDrive flippyEncoderDrive;
    FlippyDriver flippyDriver;

    public void DriveWhileWaiting(double timeoutMS, boolean waitingForLift, boolean waitingForFlippy)
    {
        float leftStickY;
        float rightStickY;
        float leftTrigger;
        float rightTrigger;

        System.out.println("ValleyX: Entering DriveWhileWaiting");
        System.out.println("ValleyX: Is running " + liftEncoderDrive.IsRunning());
        runtime_.reset();

        while (opModeIsActive() && (runtime_.milliseconds() < timeoutMS) &&
                ((liftEncoderDrive.IsRunning() || waitingForLift) ||
                        (flippyEncoderDrive.IsRunning() || waitingForFlippy)))
                {
                    if (liftEncoderDrive.IsRunning()) {
                        System.out.println("ValleyX: In checking for lift done");
                if (liftEncoderDrive.IsActionDone()) {
                    liftEncoderDrive.StopAction();

                    //TODO may need to see if going up and only turn these if so
                    robot.rightIntake.setPower(0.0);
                    robot.leftIntake.setPower(0.0);

                    System.out.println("ValleyX: Lift is done");
                    break;
                }
            }

            if (flippyEncoderDrive.IsRunning()) {
                System.out.println("ValleyX: In checking for flippy done");
                if (flippyEncoderDrive.IsActionDone()) {
                    flippyEncoderDrive.StopAction();

                    //TODO may need to see if going up and only turn these if so
                    //robot.rightIntake.setPower(0.0);
                    //robot.leftIntake.setPower(0.0);

                    System.out.println("ValleyX: Flippy is done");
                    break;
                }
            }


            //driving forward/backward
            leftStickY = -gamepad1.left_stick_y; // game pad says up is neg
            //telemetry.addData("leftY", leftStickY);
            rightStickY = -gamepad1.right_stick_y; // game pad says up is neg
            //telemetry.addData("rightY", rightStickY);
            //telemetry.update();

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;

            if (rightTrigger == 0 && leftTrigger == 0) {
                robot.leftFrontDrive.setPower(leftStickY);
                robot.leftBackDrive.setPower(leftStickY);
                robot.rightFrontDrive.setPower(rightStickY);
                robot.rightBackDrive.setPower(rightStickY);
            } else {
                //strafing right
                if (rightTrigger > 0) {
                    robot.leftFrontDrive.setPower(-rightTrigger); // +
                    robot.leftBackDrive.setPower(rightTrigger); // -
                    robot.rightFrontDrive.setPower(rightTrigger); // -
                    robot.rightBackDrive.setPower(-rightTrigger); // +
                } else if (leftTrigger == 0) {
                    robot.leftFrontDrive.setPower(0);
                    robot.leftBackDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.rightBackDrive.setPower(0);
                }

                //strafing left
                if (leftTrigger > 0) {
                    robot.leftFrontDrive.setPower(leftTrigger); // -
                    robot.leftBackDrive.setPower(-leftTrigger); // +
                    robot.rightFrontDrive.setPower(-leftTrigger); // +
                    robot.rightBackDrive.setPower(leftTrigger); // -
                } else if (rightTrigger == 0) {
                    robot.leftFrontDrive.setPower(0);
                    robot.leftBackDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.rightBackDrive.setPower(0);
                }
            }
        }
        if (flippyEncoderDrive.IsRunning())
        {
            flippyEncoderDrive.StopAction();
        }
        if (liftEncoderDrive.IsRunning())
        {
            liftEncoderDrive.StopAction();
        }

    }

    public boolean LiftTouchIsPressed()
    {
        return !robot.touchlift.getState();
    }
    public boolean FlippyTouchIsPressed()
    {
        return !robot.touchFlippy.getState();
    }

    @Override
    public void runOpMode()
    {
        robot = new RobotHardware(hardwareMap, this);
        liftEncoderDrive = new LiftEncoderDrive(robot);
        runtime_ = new ElapsedTime();
        flippyDriver = new FlippyDriver(robot);
        flippyEncoderDrive = new flippyEncoderDrive(robot);

        float leftStickY;
        float rightStickY;
        float leftTrigger;
        float rightTrigger;
        float leftTrigger2;
        float rightTrigger2;

        double ResetLiftHeight = 19;
        double HeightAfterDrop = 4;
        double GrabLiftHeight = 4.25;
        double GrabLiftExHeight = 7;
        double PlacingLiftHeight = 27;
        double arm0 = 0.87; // reset arm pos
        double ArmM = 0.9;
        double ArmA = 0.82; // grab block arm pos
        double ArmX = 0.36; // placing at 0 degrees arm pos
        double ArmY = 0.22; // placing 90 degrees back
        double ArmB = 0.32; // placing at 90 degrees front
        double claw0 = 0.40; // reset position
        double claw1 = 0.07; // y other position for placing at 90 degrees
        double claw2 = 0.77; // b placing

        double clawopeninside = 0.52;
        double platformyFlat = 0.57;
        double platformyDown = 0.28; //26

        double liftPosition = 0;

        boolean isClawOpen;

        //robot.swingy.setPosition(arm0);
        //robot.twistyClaw.setPosition(claw0);
        //robot.clawy.setPosition(clawopeninside);
        //robot.platformy.setPosition(platformyFlat);


        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)

        robot.clawy.setPosition(0.1); // open
        isClawOpen = false;

        waitForStart();
        //flippyEncoderDrive.MoveToEncoderValue(1.0, 0.05, 5, true);

        robot.platformy.setPosition(platformyDown);
        //liftEncoderDrive.MoveToEncoderValue(0.6, 10, 5, true);

        boolean flippystateIsPressed = false;
        boolean liftstateIsPressed = false;

        while (opModeIsActive())
        {
            //System.out.println("ValleyX flippy" + FlippyTouchIsPressed());
            //System.out.println("ValleyX lift" + LiftTouchIsPressed());

            if (FlippyTouchIsPressed() && !flippystateIsPressed)
            {
                flippystateIsPressed = true;
                System.out.println("ValleyX flippystateIsPressed = true");
                if (gamepad2.right_stick_y > 0) {
                    flippyEncoderDrive.ResetEncoder();
                }
            }
            else if (!FlippyTouchIsPressed() && flippystateIsPressed) {
                flippystateIsPressed = false;
                System.out.println("ValleyX flippystateIsPressed = false");
            }

            if (LiftTouchIsPressed() && !liftstateIsPressed)
            {
                liftstateIsPressed = true;
                System.out.println("ValleyX liftstateIsPressed = true");
                liftEncoderDrive.ResetEncoder();
            }
            else if (!LiftTouchIsPressed() && liftstateIsPressed)
            {
                liftstateIsPressed = false;
                System.out.println("ValleyX liftstateIsPressed = false");
            }

            /////////////////////////////GAMEPAD1//////////////////////////////////

            //driving forward/backward
            leftStickY = -gamepad1.left_stick_y; // game pad says up is neg
            //telemetry.addData("leftY", leftStickY);
            rightStickY = -gamepad1.right_stick_y; // game pad says up is neg
            //telemetry.addData("rightY", rightStickY);
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
            else {
                //strafing right
                if (rightTrigger > 0) {
                    robot.leftFrontDrive.setPower(-rightTrigger); // +
                    robot.leftBackDrive.setPower(rightTrigger); // -
                    robot.rightFrontDrive.setPower(rightTrigger); // -
                    robot.rightBackDrive.setPower(-rightTrigger); // +
                } else if (leftTrigger == 0) {
                    robot.leftFrontDrive.setPower(0);
                    robot.leftBackDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.rightBackDrive.setPower(0);
                }

                //strafing left
                if (leftTrigger > 0) {
                    robot.leftFrontDrive.setPower(leftTrigger); // -
                    robot.leftBackDrive.setPower(-leftTrigger); // +
                    robot.rightFrontDrive.setPower(-leftTrigger); // +
                    robot.rightBackDrive.setPower(leftTrigger); // -
                } else if (rightTrigger == 0) {
                    robot.leftFrontDrive.setPower(0);
                    robot.leftBackDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.rightBackDrive.setPower(0);
                }
            }
            // foundation grabbers
            if (gamepad1.right_bumper) // down grabbers
            {
                robot.rightGrabber.setPosition(0.20);
                robot.leftGrabber.setPosition(0.15);
            }
            else if (gamepad1.left_bumper) // up grabbers
            {
                robot.rightGrabber.setPosition(0.65);
                robot.leftGrabber.setPosition(0.60);
            }

/*
            if (gamepad1.y)
            {
                //robot.clawy.setPosition(clawopen);
                double currentPosition = liftEncoderDrive.CurrentEncoderPosition();
                liftEncoderDrive.MoveToEncoderValue(1.0, currentPosition + HeightAfterDrop, 5, false);
            }

 */
            if (gamepad1.x)
            {
                robot.platformy.setPosition(platformyFlat);
            }

            /*
            if (gamepad1.a)
            {
                //flippyDriver.GoToPosition(1.0, 1.0, 0);
                flippyEncoderDrive.MoveToEncoderValue(.8,.25,5,false);
            }

            if (gamepad1.b)
            {
                //flippyDriver.GoToPosition(2.0, 1.0, 0.5);
                flippyEncoderDrive.MoveToEncoderValue(.8,.40,5,false);

            }

             */



            /////////////////////////////GAMEPAD2//////////////////////////////////

            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            //double liftPosition = liftEncoderDrive.CurrentEncoderPosition();

            leftTrigger2 = gamepad2.left_trigger;
            rightTrigger2 = gamepad2.right_trigger;

            double flippyIn = 0.0;
            double flippyOut = 0.55;

            double oneLevel = 5;

            double maxlevel = 34;

            double clawopen = 0.1; //0.0
            double clawclose = 0.45; //0.37 //0.55

            double capstoneDown = 0.6;
            double capstoneUp = 0.0;

            // moving up or down levels at a time
            if (gamepad2.dpad_up)
            {
                if ((liftPosition + oneLevel) < maxlevel)
                {
                   if (liftPosition == 0.0)
                   {
                       /*
                       robot.rightIntake.setPower(1.0);
                       robot.leftIntake.setPower(-1.0);
                       robot.platformy.setPosition(platformyFlat);
                       //sleep(1000);
                       DriveWhileWaiting(300, false, false);
                       robot.clawy.setPosition(clawclose);
                       isClawOpen = false;
                       DriveWhileWaiting(300, false, false);

                        */
                   }
                    flippyEncoderDrive.MoveToEncoderValue(1.0, 0.15, 5, false); //0.2
                    DriveWhileWaiting(300, false, true);

                    liftPosition = liftPosition + oneLevel;

                    liftEncoderDrive.MoveToEncoderValue(0.9, liftPosition, 5, false);
                    DriveWhileWaiting(300, true, false);
                    //robot.rightIntake.setPower(0.0);
                    //robot.leftIntake.setPower(0.0);

                }
            }
            if (gamepad2.dpad_down)
            {
                if ((liftPosition - oneLevel) >= 0.0)
                {
                    liftPosition = liftPosition - oneLevel;

                    liftEncoderDrive.MoveToEncoderValue(0.6, liftPosition,5, false);
                    DriveWhileWaiting(300, true, false);
                }
            }

            // reset
            if (gamepad2.a)
            {
                telemetry.addData("ValleyX reseting ", 23);
                robot.clawy.setPosition(clawclose);
                isClawOpen = false;
                //sleep(600);
                DriveWhileWaiting(400, false, false);
                //flippyEncoderDrive.MoveToEncoderValue(0.2, flippyOut, 5, false);
                liftPosition = 0;
                flippyEncoderDrive.MoveToEncoderValue(1.0, flippyIn, 2, false);
                liftEncoderDrive.MoveToEncoderValue(0.8, liftPosition, 2, false);
                DriveWhileWaiting(1200, false, true);
                DriveWhileWaiting(700, true, false);
                //sleep(600);
                //robot.clawy.setPosition(clawopen);
                //JM just took out 4:04 2-5-2020
                //robot.platformy.setPosition(platformyDown);
            }

            // flipping the arm outside the bot (level one/swing out)
            if (gamepad2.y)
            {
               /*
                if (liftPosition == 0)
                {

                    robot.rightIntake.setPower(1.0);
                    robot.leftIntake.setPower(-1.0);
                    robot.platformy.setPosition(platformyFlat);
                    //sleep(400);
                    DriveWhileWaiting(400, false, false);
                    robot.clawy.setPosition(clawclose);

                    robot.clawy.setPosition(clawclose);
                    isClawOpen = false;
                    //sleep(400);
                    DriveWhileWaiting(400, false, false);

                }

                */

                robot.clawy.setPosition(clawclose);
                isClawOpen = false;
                flippyEncoderDrive.MoveToEncoderValue(1.0, flippyOut, 5, false);
                DriveWhileWaiting(1200,false, true);
                robot.rightIntake.setPower(0.0);
                robot.leftIntake.setPower(0.0);
            }

            // open/close the claw
            if (leftBumper)
            {
                robot.clawy.setPosition(clawopen);
                DriveWhileWaiting(300, false, false);
                if (((liftPosition + oneLevel) < maxlevel) && (isClawOpen == false))
                {
                    //liftPosition = liftPosition + oneLevel;

                    //liftEncoderDrive.MoveToEncoderValue(0.9, liftPosition, 5, false);
                    telemetry.addData("ValleyX moving arm up a little bit ", 23);
                    //flippyEncoderDrive.MoveToEncoderValue(1.0, 0.55, 5, false);
                    //DriveWhileWaiting(100, false, true);
                    //robot.clawy.setPosition(clawclose);
                }
                isClawOpen = true;
            }
            if (rightBumper)
            {
                robot.clawy.setPosition(clawclose);
                isClawOpen = false;
            }

            // intake
            if (rightTrigger2 > 0)
            {
                //robot.platformy.setPosition(platformyDown);
                if (robot.bucketLazery.getDistance(DistanceUnit.INCH) < 2)
                {
                    DriveWhileWaiting(400, false, false);
                    robot.platformy.setPosition(platformyFlat);
                    DriveWhileWaiting(600, false, false);
                    robot.clawy.setPosition(clawclose);
                    isClawOpen = false;
                }
                else
                {
                    robot.platformy.setPosition(platformyDown);
                }
                //robot.clawy.setPosition(clawopen);
                robot.rightIntake.setPower(rightTrigger2);
                robot.leftIntake.setPower(-rightTrigger2);
            }
            else if (leftTrigger2 > 0) // out
            {
                robot.platformy.setPosition(platformyDown);
                robot.clawy.setPosition(clawopen);
                isClawOpen = true;
                robot.rightIntake.setPower(-leftTrigger2);
                robot.leftIntake.setPower(leftTrigger2);
            }
            else
            {
                robot.rightIntake.setPower(0);
                robot.leftIntake.setPower(0);
            }

            robot.lift.setPower(-gamepad2.left_stick_y);
            if (gamepad2.right_stick_y != 0.0) {
                robot.clawy.setPosition(clawclose);
                flippyEncoderDrive.StopAction();
                robot.flippy.setPower(-gamepad2.right_stick_y);
            }

            // capstone button
            if (gamepad2.b)
            {
                robot.rightIntake.setPower(rightTrigger2);
                robot.leftIntake.setPower(-rightTrigger2);
                robot.clawy.setPosition(clawopen);
                flippyEncoderDrive.MoveToEncoderValue(1.0, 0.3, 5, false);
                DriveWhileWaiting(600,false, true);
                robot.capstoneServo.setPosition(capstoneDown);
                sleep(300);
                robot.capstoneServo.setPosition(capstoneUp);
                sleep(300);
                flippyEncoderDrive.MoveToEncoderValue(1.0, 0.0, 5, false);
                DriveWhileWaiting(600,false, true);
            }

/*
            // reset everything
            if (gamepad2.x)
            {
                robot.platformy.setPosition(platformyDown);
                robot.clawy.setPosition(clawclose);
                flippyEncoderDrive.MoveToEncoderValue(0.3, flippyReset, 5, true);

            }

 */
/*
            leftTrigger2 = gamepad2.left_trigger;
            rightTrigger2 = gamepad2.right_trigger;

            telemetry.addData("lift encoder value ", liftEncoderDrive.CurrentEncoderPosition());
            //telemetry.update();

            // Reset button
            if (gamepad2.right_bumper)
            {
                robot.leftIntake.setPower(-1.0);
                robot.rightIntake.setPower(-1.0);

                //Safety Protocol for stupidity
                robot.platformy.setPosition(platformyFlat);
                //robot.twistyClaw.setPosition(claw0);
                //robot.clawy.setPosition(clawclose);
                //sleep(500);
                DriveWhileWaiting(500);
                //Raise/Lower Lift to clear base of robot
                System.out.println("ValleyX: Raising to lift height");
                liftEncoderDrive.MoveToEncoderValue(0.8, ResetLiftHeight, 5, false);
                System.out.println("ValleyX: Enter drive while waiting");
                //sleep(500);
                DriveWhileWaiting(500);
                System.out.println("ValleyX: Done with raising");

                //robot.swingy.setPosition(ArmM);
                DriveWhileWaiting(1000);
                //Lower Lift for Reset
                liftEncoderDrive.MoveToEncoderValue(0.6, 0.1, 5, false);
                telemetry.addData("ValleyX lift encoder", liftEncoderDrive.CurrentEncoderPosition());
                //telemetry.update();
                //sleep(1000);
                DriveWhileWaiting(1000);
                //robot.swingy.setPosition(arm0);

                //sleep(500);
                DriveWhileWaiting((500));
                //robot.clawy.setPosition(clawopeninside);
                robot.platformy.setPosition(platformyDown);

                robot.leftIntake.setPower(0.0);
                robot.rightIntake.setPower(0.0);
            }

            // grab block
            if (gamepad2.a)
            {
                //robot.twistyClaw.setPosition(claw0);
                robot.leftIntake.setPower(-1.0);
                robot.rightIntake.setPower(1.0);
                //robot.swingy.setPosition(ArmM);
                //sleep(500);
                DriveWhileWaiting(500);
                robot.platformy.setPosition(platformyFlat);
                //sleep(200);
                DriveWhileWaiting(200);
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
                liftEncoderDrive.MoveToEncoderValue(1.0, GrabLiftExHeight, 5, false);
                //sleep(500);
                DriveWhileWaiting(500);
                ///robot.clawy.setPosition(clawopeninside);
                //sleep(200);
                DriveWhileWaiting(200);
                //robot.swingy.setPosition(ArmA);
                //sleep(700);
                DriveWhileWaiting(700);
                liftEncoderDrive.MoveToEncoderValue(0.6, GrabLiftHeight, 5, false);
                //sleep(500);
                DriveWhileWaiting(500);
                //robot.clawy.setPosition(clawclose);
                DriveWhileWaiting(700);
                //sleep(700);
                //robot.swingy.setPosition(ArmM);

            }

            // placing block @ 0 degrees
            if (gamepad2.x)
            {
                //robot.twistyClaw.setPosition(claw0);
                //robot.clawy.setPosition(clawclose);
                //sleep(500);
                DriveWhileWaiting(500);
                liftEncoderDrive.MoveToEncoderValue(1.0, PlacingLiftHeight, 5, false);
                DriveWhileWaiting(1);
                //robot.swingy.setPosition(ArmX);
                robot.platformy.setPosition(platformyDown);
            }

            // placing block at 90 degrees (back)
            if (gamepad2.y)
            {
                //.twistyClaw.setPosition(claw0);
                //robot.clawy.setPosition(clawclose);
                sleep(500);
                liftEncoderDrive.MoveToEncoderValue(1.0, PlacingLiftHeight, 5, false);
                DriveWhileWaiting(1);
                //robot.swingy.setPosition(ArmY);
                //sleep(1000);
                DriveWhileWaiting(1000);
                //robot.twistyClaw.setPosition(claw1);
                robot.platformy.setPosition(platformyDown);
            }

            // placing block at 90 degrees (front)
            if (gamepad2.b)
            {
                //robot.twistyClaw.setPosition(claw0);
                //robot.clawy.setPosition(clawclose);
                //sleep(500);
                DriveWhileWaiting(500);
                liftEncoderDrive.MoveToEncoderValue(1.0, PlacingLiftHeight, 5, false);
                DriveWhileWaiting(1);
                //robot.swingy.setPosition(ArmB);
                //sleep(1000);
                DriveWhileWaiting(1000);
                //robot.twistyClaw.setPosition(claw2);
                robot.platformy.setPosition(platformyDown);
            }

            // intake
            telemetry.addData("right trigger2 ", rightTrigger2);
            telemetry.addData("left trigger2 ", leftTrigger2);
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

            // lift power on a lever E
            double leftStickY2 = -gamepad2.left_stick_y;
            double currentLiftPosition = liftEncoderDrive.CurrentEncoderPosition();
            double bottomCutOff = 0.1;
            double topCutOff = 33;

            if (!liftEncoderDrive.IsRunning()) {
                if ((leftStickY2 < 0) && (currentLiftPosition > bottomCutOff)) {
                    robot.lift.setPower(leftStickY2);
                } else if ((leftStickY2 > 0) && (currentLiftPosition < topCutOff)) {
                    robot.lift.setPower(leftStickY2);
                } else {
                    robot.lift.setPower(0);
                }
            }
            else
            {
                if (liftEncoderDrive.IsActionDone()) {
                    liftEncoderDrive.StopAction();
                }
            }
*/
           // telemetry.update();
            idle();
        }
    }
}
