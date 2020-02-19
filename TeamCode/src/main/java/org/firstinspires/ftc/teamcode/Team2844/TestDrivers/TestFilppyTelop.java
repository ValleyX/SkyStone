package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.flippyEncoderDrive;

@TeleOp(name="TeleOp: Test Flippy", group="Test")
//@Disabled

public class TestFilppyTelop extends LinearOpMode
{

    //RobotHardware robot;
    //flippyEncoderDrive flippyEncoderDrive;
    DcMotor flippy;

    @Override
    public void runOpMode() {
        //robot = new RobotHardware(hardwareMap, this);
        //flippyEncoderDrive = new flippyEncoderDrive(robot);
        flippy = hardwareMap.get(DcMotor.class, "flippyNew"); // secondary hub motor 3

        flippy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flippy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        //flippyEncoderDrive.MoveToEncoderValue(1.0, 0.5,5, true);

        while (opModeIsActive()) {
            flippy.setPower(gamepad2.right_stick_y);
            telemetry.addData("Flippy Encoder", "current encoder value %7d",
                    flippy.getCurrentPosition());
            //telemetry.addData("Flippy Encoder", "running to %7d",              position);
            telemetry.update();
        }
    }
}
