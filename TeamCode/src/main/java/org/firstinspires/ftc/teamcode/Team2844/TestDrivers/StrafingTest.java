package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@Autonomous(name="Test: Strafing", group="Test")
@Disabled
public class StrafingTest extends LinearOpMode
{
    public void runOpMode()
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        StrafingEncoderDrive strafe = new StrafingEncoderDrive(robot);

        waitForStart();

        strafe.Strafe(0.6, -12, 40, true);
        //sleep(2000);
        strafe.Strafe(0.6, 12, 40, true);

/*
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.leftFrontDrive.setPower(0.6);
        robot.leftBackDrive.setPower(-0.6);
        robot.rightFrontDrive.setPower(-0.6);
        robot.rightBackDrive.setPower(0.6);

        sleep(2000);

        robot.leftFrontDrive.setPower(0.0);
        robot.leftBackDrive.setPower(0.0);
        robot.rightFrontDrive.setPower(0.0);
        robot.rightBackDrive.setPower(0.0);

 */
    }
}
