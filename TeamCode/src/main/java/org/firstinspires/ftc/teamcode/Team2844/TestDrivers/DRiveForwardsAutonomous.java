package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@Autonomous(name="DRive forwards autonomous", group="Test")
@Disabled

public class DRiveForwardsAutonomous extends LinearOpMode
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

        encoderDriveHeading.StartAction(0.6, 10, 0, 5, true);
    }
}