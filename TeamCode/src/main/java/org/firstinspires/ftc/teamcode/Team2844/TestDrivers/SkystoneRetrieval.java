package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.ColorDriver;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;


@Autonomous(name = "Test: SkystoneRetrieval", group ="Test")
public class SkystoneRetrieval extends LinearOpMode {
    ColorDriver colorDriver;
    EncoderDrive encoderDrive;
    TestRobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TestRobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);


        waitForStart();
        while (opModeIsActive()){
            encoderDrive.StartAction(.5, -43, -43, 5, true);
            while (colorDriver.isSeen() && colorDriver.isYellow()){
                Strafing.Strafe(.5, );
            }
        }
    }
}
