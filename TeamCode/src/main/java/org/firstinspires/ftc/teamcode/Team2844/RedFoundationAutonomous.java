package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.LiftEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;

@Autonomous(name="Red Foundation Autonomous", group="Test")
//@Disabled

public class RedFoundationAutonomous extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        // for blue side
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);
        //LiftEncoderDrive liftEncoderDrive = new LiftEncoderDrive(robot);
        EncoderDriveHeading encoderDriveHeading = new EncoderDriveHeading(robot);
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        // Wait for the game to start (driver presses PLAY)
        System.out.println("ValleyX: Waiting for Start");
        robot.rightGrabber.setPosition(0.65);
        robot.leftGrabber.setPosition(0.60);

        waitForStart();

        System.out.println("ValleyX: Starting...");


        System.out.println("ValleyX: Move backwards 28 inches ");
//        encoderDrive.StartAction(0.4, -29.5, -29.5, 5.0, true);
        encoderDriveHeading.StartAction(0.6, -27, 0, 5, true); //-32
        encoderDriveHeading.StartAction(0.2, -4.5, 0, 5, true);

        //System.out.println("ValleyX strafing right 9 inches");
        //Strafing.Strafe(0.6, 9, 5, true);

        // raise lift so that grabbers can come down
        // liftEncoderDrive.MoveToEncoderValue(0.6, 6, 5, true);

        // latch onto foundation
        System.out.println("ValleyX lowering grabbers");
        robot.rightGrabber.setPosition(0.20);
        robot.leftGrabber.setPosition(0.15);

        // drag foundation back
        sleep(1000);
        //System.out.println("ValleyX: Move forwards 30 inches ");
        //encoderDrive.StartAction(0.6, 30, 30, 5.0, true);

        encoderDriveHeading.StartAction(0.9, 13, 0, 5, true);
        rotatePrecise.RotatePrecise(90, 2, 0.6, 0.3, 2);
        encoderDriveHeading.StartAction(1.0, -5, 90, 5, true);

        // unlatch from foundation
        System.out.println("ValleyX raising grabbers");
        robot.rightGrabber.setPosition(0.65);
        robot.leftGrabber.setPosition(0.60);

        sleep(1000);


        // strafe right
        System.out.println("ValleyX: Strafe left 35 inches ");
        Strafing.Strafe(0.9, -17, 5.0, true);
/*
        // forwards
        System.out.println("ValleyX: Move backwards 19 inches ");
        encoderDrive.StartAction(0.6, -19, -19, 5.0, true);

        // push foundation into building site
        System.out.println("ValleyX: Strafe right 15 inches ");
        Strafing.Strafe(0.6, 15, 5.0, true);
*/
        // park under SkyBridge
        System.out.println("ValleyX: Move forwards 22 inches ");
        //encoderDrive.StartAction(0.6, 22, 22, 5.0, true);
        encoderDriveHeading.StartAction(1.0, 30, 90, 5, true);

        System.out.println("ValleyX: Strafe left 30 inches ");
        Strafing.Strafe(0.9, -30, 5.0, true);


    }
}