package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;

@Autonomous(name="Blue Foundation Autonomous", group="Test")
//@Disabled

public class BlueFoundationAutonomous extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        // for blue side
        TestRobotHardware robot = new TestRobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);

        // Wait for the game to start (driver presses PLAY)
        System.out.println("ValleyX: Waiting for Start");
        waitForStart();

        System.out.println("ValleyX: Starting...");

        robot.leftGrabber.setPosition(0.0);
        robot.rightGrabber.setPosition(0.0);

        System.out.println("ValleyX: Move backwards 28 inches ");
        encoderDrive.StartAction(0.6, -28, -28, 5.0, true);

        System.out.println("ValleyX strafing right 9 inches");
        Strafing.Strafe(0.6, 9, 5, true);

        // latch onto foundation
        System.out.println("ValleyX lowering grabbers");
        robot.leftGrabber.setPosition(0.75);
        robot.rightGrabber.setPosition(0.75);

        // drag foundation back
        sleep(1000);
        System.out.println("ValleyX: Move forwards 30 inches ");
        encoderDrive.StartAction(0.6, 30, 30, 5.0, true);

        // unlatch from foundation
        System.out.println("ValleyX raising grabbers");
        robot.leftGrabber.setPosition(0.0);
        robot.rightGrabber.setPosition(0.0);

        sleep(1000);

        // strafe right
        System.out.println("ValleyX: Strafe left 35 inches ");
        Strafing.Strafe(0.6, -35, 5.0, true);

        // forwards
        System.out.println("ValleyX: Move backwards 19 inches ");
        encoderDrive.StartAction(0.6, -19, -19, 5.0, true);

        // push foundation into building site
        System.out.println("ValleyX: Strafe right 15 inches ");
        Strafing.Strafe(0.6, 15, 5.0, true);

        // park under SkyBridge
        System.out.println("ValleyX: Move forwards 22 inches ");
        encoderDrive.StartAction(0.6, 22, 22, 5.0, true);

        System.out.println("ValleyX: Strafe left 30 inches ");
        Strafing.Strafe(0.6, -30, 5.0, true);
    }
}