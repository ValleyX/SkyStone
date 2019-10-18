package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@TeleOp(name="Test: Test Rotate Precise", group="Test")
//@Disabled

public class TestRotatePrecise extends LinearOpMode
{
    //@Override
    public void runOpMode()
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        RotatePrecise rotatePrecise = new RotatePrecise(robot);

        //sleep(2000);
        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Rotate 90 degrees");

        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a) {
                rotatePrecise.RotatePrecise(90, 2, 0.2, 0.3, 5);
                sleep(2000);
                telemetry.addData("gyrotarget ", 90);
                telemetry.update();
            }

            if (gamepad1.b)
            {
                rotatePrecise.RotatePrecise(-90, 2, 0.2, 0.3, 5);
                sleep(2000);
                telemetry.addData("gyrotarget ", 90);
                telemetry.update();
            }
        }

        /*
        telemetry.addData("gyrotarget ", -90);
        telemetry.update();
        rotatePrecise.RotatePrecise(-90, 1, 0.2, 0.3, 5);
        sleep(2000);

        telemetry.addData("gyrotarget ", 270);
        telemetry.update();
        rotatePrecise.RotatePrecise(270, 1, 0.2, 0.3, 5);
        sleep(2000);

        telemetry.addData("gyrotarget ", 45);
        telemetry.update();
        rotatePrecise.RotatePrecise(45, 1, 0.2, 0.3, 5);
        sleep(2000);

        telemetry.addData("gyrotarget ", 15);
        telemetry.update();
        rotatePrecise.RotatePrecise(15, 1, 0.2, 0.3, 5);
        sleep(2000);

        telemetry.addData("gyrotarget ", 20);
        telemetry.update();
        rotatePrecise.RotatePrecise(20, 1, 0.2, 0.3, 5);
        sleep(2000);

        telemetry.addData("gyrotarget ", 10);
        telemetry.update();
        rotatePrecise.RotatePrecise(10, 1, 0.2, 0.3, 5);
        sleep(2000);
         */
    }
}

