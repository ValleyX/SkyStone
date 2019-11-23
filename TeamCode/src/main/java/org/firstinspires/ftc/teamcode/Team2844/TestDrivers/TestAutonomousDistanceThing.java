package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.DriveTo;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;

@Autonomous(name="Test: Autonomous Distance Thing", group="Test")
//@Disabled

public class TestAutonomousDistanceThing extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();

        TestRobotHardware robot = new TestRobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        DriveTo driveTo = new DriveTo(robot, encoderDrive);

        double distanceMoved;

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive.StartAction(1.0, -22, -22, 5, true);
        driveTo.StartAction(1.0, 2, 5, true);

        encoderDrive.StartAction(1.0, 5, 5, 5, true);
        rotatePrecise.RotatePrecise(-90, 1, 0.2, 0.3, 5);

        encoderDrive.StartAction(1.0, -50, -50, 5, true);
        distanceMoved = driveTo.StartAction(1.0, 2, 5, true);

        encoderDrive.StartAction(1.0, 50+distanceMoved, 50+distanceMoved, 5, true);
        rotatePrecise.RotatePrecise(90, 1, 0.2, 0.3, 5);

        driveTo.StartAction(1.0, 2, 5, true);
    }
}
