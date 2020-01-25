package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.FlippyDriver;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@Autonomous(name="MoveFlippy", group="Test")

public class MoveFlippy extends LinearOpMode
{
    public void runOpMode()
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        FlippyDriver flippy = new FlippyDriver(robot);

        waitForStart();

        flippy.GoToPosition(0.2, 0.6);
        robot.flippy.setPower(0.15);
        /*
        System.out.println("ValleyX platform servo 1 " + robot.platformy.getPosition());
        robot.platformy.setPosition(0.35);
        sleep(1000);
        System.out.println("ValleyX platform servo 2 " + robot.platformy.getPosition());

         */
        robot.rightIntake.setPower(1.0);
        robot.leftIntake.setPower(-1.0);
        sleep(1000);
        robot.rightIntake.setPower(0.0);
        robot.leftIntake.setPower(0.0);

        sleep(10000);
    }
}
