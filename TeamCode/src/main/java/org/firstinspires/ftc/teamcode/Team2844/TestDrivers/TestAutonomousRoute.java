package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@TeleOp(name="Test: TestAutonomousRoute", group="Test")
@Disabled

public class TestAutonomousRoute extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        // for blue side
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // latch onto foundation
        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Move forward 30 inches ");
        encoderDrive.StartAction(0.6, 30, 30, 5.0, true);

        while (!gamepad1.a)
        {
            idle();
        }

        // drag foundation back
        System.out.println("ValleyX: Move backwards 30 inches ");
        encoderDrive.StartAction(0.6, -30, -30, 5.0, true);

        while (!gamepad1.b)
        {
            idle();
        }

        // strafe right
        System.out.println("ValleyX: Strafe right 21 inches ");
        Strafing.Strafe(0.6, 22, 5.0, true);

        // forwards
        System.out.println("ValleyX: Move forwards 17 inches ");
        encoderDrive.StartAction(0.6, 19, 17, 5.0, true);

        // push foundation into building site
        System.out.println("ValleyX: Strafe left 9.5 inches ");
        Strafing.Strafe(0.6, -11, 5.0, true);

        // park under SkyBridge
        System.out.println("ValleyX: Strafe Right 28.5 inches ");
        Strafing.Strafe(0.6, 30, 5.0, true);
    }
}