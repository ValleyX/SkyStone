package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;


@TeleOp(name = "BlinkTimery", group = "Lights")

public class BlinkTimery extends LinearOpMode{

    private ElapsedTime runtime_;
    private RobotHardware robot_;
    RevBlinkinLedDriver blinky;

    @Override
    public void runOpMode() throws InterruptedException {
        blinky = hardwareMap.get(RevBlinkinLedDriver.class, "Blinker_Revy");

        runtime_ = new ElapsedTime();

        while (opModeIsActive()){
            blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);

            if (runtime_.seconds() > 115) {
                robot_.blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
            } else if (runtime_.seconds() > 95){
                robot_.blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (runtime_.seconds() > 90) {
                robot_.blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }


        }
    }
}

