package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardwarePushbot;

@TeleOp(name="test: Test Gamepad Servo", group="Test")
@Disabled
public class Servo1TestLukeisCoolerthanJonothan extends LinearOpMode {

    public static final double Latch_UP_POSITION = -0.25;
    public static final double Latch_DOWN_POSITION = 0.70;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardwarePushbot robot = new RobotHardwarePushbot(hardwareMap, this);

        System.out.println("ValleyX waiting for start");
        waitForStart();
        System.out.println("ValleyX starting");

        while (opModeIsActive()) {
            boolean DpadBackLatchLeft = gamepad1.dpad_left;
            boolean DpadBackLatchRight = gamepad1.dpad_right;

            telemetry.addData("DpadBackLatchLeft=%d", DpadBackLatchLeft);
            telemetry.addData("DpadBackLatchRight=%d", DpadBackLatchRight);

            if ((DpadBackLatchLeft == true) && (DpadBackLatchRight == false)) {
                robot.BackLatch.setPosition(Latch_UP_POSITION);
                System.out.println("ValleyX LatchLeftPress");
            }

            if ((DpadBackLatchRight == true) && (DpadBackLatchLeft == false)) {
                robot.BackLatch.setPosition(Latch_DOWN_POSITION);
                System.out.println("ValleyX Latch Right Press");
            }
        }
    }
}