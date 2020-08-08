package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="Test : Test Gamepad", group="Test")
public abstract class Gamepad_Test  extends LinearOpMode {

    public void runOpmode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive()) {
            double joystickY = gamepad1.left_stick_y;
            double joystickX = gamepad1.left_stick_x;

            telemetry.addData("joystickX = %d", joystickX);
            telemetry.addData("joystickY = %d", joystickY);
            telemetry.update();


        }
    }
}
