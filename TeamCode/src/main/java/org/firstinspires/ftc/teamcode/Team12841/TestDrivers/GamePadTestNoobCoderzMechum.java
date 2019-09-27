package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="test: Test Gamepad mecam", group="Test")

public class GamePadTestNoobCoderzMechum extends LinearOpMode {
    DcMotor LeftFrontMotor;
    DcMotor RightFrontMotor;
    DcMotor LeftBackMotor;
    DcMotor RightBackMotor;

    @Override
    public void runOpMode() throws InterruptedException {


        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LFmotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RFmotor");
        LeftBackMotor = hardwareMap.get(DcMotor.class, "LBmotor");
        RightBackMotor = hardwareMap.get(DcMotor.class, "RBmotor");
        RightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double joystickLeftX = gamepad1.left_stick_y;
            double joystickRightY = gamepad1.right_stick_y;

            double RightButton = gamepad1.right_trigger;
            double LeftButton = gamepad1.left_trigger;

            telemetry.addData("joystickLeftX=%d", joystickLeftX);
            telemetry.addData("joystickRightY=%d", joystickRightY);

            if ((RightButton == 0) && (LeftButton == 0)) {
                LeftFrontMotor.setPower(joystickRightY);
                RightFrontMotor.setPower(joystickLeftX);
                LeftBackMotor.setPower(joystickRightY);
                RightBackMotor.setPower(joystickLeftX);

            }

            telemetry.addData("right_trigger=%d", RightButton);
            telemetry.addData("left_trigger=%d", LeftButton);
            telemetry.update();

            LeftFrontMotor.setPower(LeftButton);
            RightFrontMotor.setPower(RightButton);
            LeftBackMotor.setPower(LeftButton);
            RightBackMotor.setPower(RightButton);



        }
    }
}