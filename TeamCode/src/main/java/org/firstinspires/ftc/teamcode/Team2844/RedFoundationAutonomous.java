package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@TeleOp(name="Autonomous: Red Foundation Autonomous", group="Test")
//@Disabled

public class RedFoundationAutonomous extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        // for red side
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);

        // Wait for the game to start (driver presses PLAY)
        System.out.println("ValleyX: Waiting for Start");
        waitForStart();

        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Move backwards 30 inches ");
        encoderDrive.StartAction(0.6, -30, -30, 5.0, true);

        // latch onto foundation
        robot.leftGrabber.setPosition(0.5);
        robot.rightGrabber.setPosition(0.5);

        /*
        while (!gamepad1.a)
        {
            idle();
        }
         */

        // drag foundation back
        System.out.println("ValleyX: Move forwards 30 inches ");
        encoderDrive.StartAction(0.6, 30, 30, 5.0, true);

        // unlatch from foundation
        robot.leftGrabber.setPosition(0);
        robot.rightGrabber.setPosition(0);

        /*
        while (!gamepad1.b)
        {
            idle();
        }
         */

        // strafe right
        System.out.println("ValleyX: Strafe right 22 inches ");
        Strafing.Strafe(0.6, 22, 5.0, true);

        // forwards
        System.out.println("ValleyX: Move backwards 19 inches ");
        encoderDrive.StartAction(0.6, -19, -19, 5.0, true);

        // push foundation into building site
        System.out.println("ValleyX: Strafe left 11 inches ");
        Strafing.Strafe(0.6, -11, 5.0, true);

        // park under SkyBridge
        System.out.println("ValleyX: Strafe right 30 inches ");
        Strafing.Strafe(0.6, 30, 5.0, true);

        System.out.println("ValleyX: Move forwards 22 inches ");
        encoderDrive.StartAction(0.6, 22, 22, 5.0, true);
    }
}