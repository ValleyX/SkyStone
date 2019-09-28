package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;

@Autonomous(name="Test: Test Rotate Precise", group="Test")
//@Disabled

public class TestRotatePrecise extends LinearOpMode
{
    //@Override
    public void runOpMode()
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        RotatePrecise rotatePrecise = new RotatePrecise(robot);

        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Rotate 90 degrees");

        rotatePrecise.RotatePrecise(90, 1, 0.2, 0.3, 5);
    }

}

