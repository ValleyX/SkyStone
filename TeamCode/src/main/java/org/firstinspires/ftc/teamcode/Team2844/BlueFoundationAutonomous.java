package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@TeleOp(name="Autonomous: Blue Foundation Autonomous", group="Test")
//@Disabled

public class BlueFoundationAutonomous extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        // for blue side
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);

        // Wait for the game to start (driver presses PLAY)
        System.out.println("ValleyX: Waiting for Start");
        waitForStart();

        System.out.println("ValleyX: Starting...");

        System.out.println("ValleyX raising grabbers");
        robot.leftGrabber.setPosition(0.0);
        robot.rightGrabber.setPosition(0.0);

        System.out.println("ValleyX: Move backwards 30 inches ");
        encoderDrive.StartAction(0.6, -30, -30, 5.0, true);

        // latch onto foundation
        /*
        while (!gamepad1.a)
        {
            idle();
        }
         */

        System.out.println("ValleyX lowering grabbers");
        robot.leftGrabber.setPosition(0.5); // find value
        robot.rightGrabber.setPosition(0.5); // find value

        // drag foundation back
        System.out.println("ValleyX: Move forwards 30 inches ");
        encoderDrive.StartAction(0.6, 30, 30, 5.0, true);

        // unlatch from foundation
        /*
        while (!gamepad1.b)
        {
            idle();
        }
         */

        System.out.println("ValleyX raising grabbers");
        robot.leftGrabber.setPosition(0); // find value
        robot.rightGrabber.setPosition(0); // find value

        // strafe right
        System.out.println("ValleyX: Strafe left 22 inches ");
        Strafing.Strafe(0.6, -22, 5.0, true);

        // forwards
        System.out.println("ValleyX: Move backwards 19 inches ");
        encoderDrive.StartAction(0.6, -19, -19, 5.0, true);

        // push foundation into building site
        System.out.println("ValleyX: Strafe right 11 inches ");
        Strafing.Strafe(0.6, 11, 5.0, true);

        // park under SkyBridge
        System.out.println("ValleyX: Strafe Right 30 inches ");
        Strafing.Strafe(0.6, 30, 5.0, true);

        System.out.println("ValleyX: Move forwards 22 inches ");
        encoderDrive.StartAction(0.6, 22, 22, 5.0, true);
    }
}