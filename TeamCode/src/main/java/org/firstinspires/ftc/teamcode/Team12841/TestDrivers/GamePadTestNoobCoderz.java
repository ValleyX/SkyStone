package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="test: Test Gamepad", group="Test")

public class GamePadTestNoobCoderz extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;


    @Override
    public void runOpMode() throws InterruptedException {


        LeftMotor = hardwareMap.get(DcMotor.class, "Lmotor");
        RightMotor = hardwareMap.get(DcMotor.class, "Rmotor");
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double joystickLeftX = gamepad1.left_stick_y;
            double joystickRightY = gamepad1.right_stick_y;

            double RightButton = gamepad1.right_trigger;
            double LeftButton = gamepad1.left_trigger;

            telemetry.addData("joystickLeftX=%d", joystickLeftX);
            telemetry.addData("joystickRightY=%d", joystickRightY);

            if ((RightButton == 0) && (LeftButton == 0)) {
                LeftMotor.setPower(joystickRightY);
                RightMotor.setPower(joystickLeftX);
            }

            telemetry.addData("right_trigger=%d", RightButton);
            telemetry.addData("left_trigger=%d", LeftButton);
            telemetry.update();

            LeftMotor.setPower(LeftButton);
            RightMotor.setPower(RightButton);

        }
    }
}