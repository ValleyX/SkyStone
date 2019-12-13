package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.LiftEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;

@Autonomous(name="Red Foundation Autonomous", group="Test")
//@Disabled

public class RedFoundationAutonomous extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        // for red side
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);
        LiftEncoderDrive liftEncoderDrive = new LiftEncoderDrive(robot);

        // Wait for the game to start (driver presses PLAY)
        System.out.println("ValleyX: Waiting for Start");
        waitForStart();

        System.out.println("ValleyX: Starting...");

        robot.leftGrabber.setPosition(0.0);
        robot.rightGrabber.setPosition(0.0);

        System.out.println("ValleyX: Move backwards 28 inches ");
        encoderDrive.StartAction(0.6, -28, -28, 5.0, true);

        System.out.println("ValleyX strafing left 9 inches");
        Strafing.Strafe(0.6, -9, 5, true);

        // raise lift so that grabbers can come down
        liftEncoderDrive.MoveToEncoderValue(0.6, 6, 5, true);

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

        // lower lift so that grabbers can come down
        liftEncoderDrive.MoveToEncoderValue(0.6, 0, 5, true);

        // strafe right
        System.out.println("ValleyX: Strafe right 30 inches ");
        Strafing.Strafe(0.6, 30, 5.0, true);

        // forwards
        System.out.println("ValleyX: Move backwards 19 inches ");
        encoderDrive.StartAction(0.6, -19, -19, 5.0, true);

        // push foundation into building site
        System.out.println("ValleyX: Strafe left 15 inches ");
        Strafing.Strafe(0.6, -15, 5.0, true);

        // park under SkyBridge
        System.out.println("ValleyX: Move forwards 22 inches ");
        encoderDrive.StartAction(0.6, 25, 25, 5.0, true);

        System.out.println("ValleyX: Strafe right 30 inches ");
        Strafing.Strafe(0.6, 30, 5.0, true);
    }
}