package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@TeleOp(name="Test: Test Meca Drive", group="Test")
//@Disabled

public class TestAutonomousRoute extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        // for blue side
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Move forward 30 inches ");
        encoderDrive.StartAction(0.6, 30, 30, 5.0, true);

        while (!gamepad1.a)
        {
            idle();
        }

        System.out.println("ValleyX: Move backwards 30 inches ");
        encoderDrive.StartAction(0.6, -30, -30, 5.0, true);

        while (!gamepad1.b)
        {
            idle();
        }

        //strafe right
    }
}