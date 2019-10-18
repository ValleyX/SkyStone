package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.GoToPosition;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.VuforiaPosition;

@TeleOp(name="Test: TestGoToPosition", group="Test")

public class TestGoToPosition extends LinearOpMode
{
    public void runOpMode() // need to test
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);
        VuforiaPosition vuforiaPosition = new VuforiaPosition(robot);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        GoToPosition GoToPosition = new GoToPosition(robot, rotatePrecise, rotateToHeading, vuforiaPosition, encoderDrive);

        waitForStart();

        GoToPosition.GoToPosition(45, 36);
    }
}
