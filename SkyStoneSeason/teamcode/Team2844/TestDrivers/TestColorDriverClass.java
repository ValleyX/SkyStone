package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.ColorDriver;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;

@TeleOp(name="Test Color Driver Class", group="Test")
public class TestColorDriverClass extends LinearOpMode {

    TestRobotHardware robot_;
    ColorDriver colorDriver;

    @Override
    public void runOpMode(){
        robot_ = new TestRobotHardware(hardwareMap, this);
        colorDriver = new ColorDriver(robot_);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("isYellow", colorDriver.isYellow());
            telemetry.addData("isSeen",colorDriver.isSeen());
            telemetry.update();
        }
    }
}
