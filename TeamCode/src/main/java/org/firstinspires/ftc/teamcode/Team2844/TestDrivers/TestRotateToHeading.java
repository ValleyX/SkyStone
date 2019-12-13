package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;

@TeleOp(name="Test: TestRotateToHeading", group="Test")
@Disabled

public class TestRotateToHeading extends LinearOpMode
{
    public void runOpMode() // need to test
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);

        waitForStart();

        System.out.println("ValleyX turning to 180 heading");
        rotateToHeading.DoIt(180);
        //rotatePrecise.RotatePrecise(90, 2, 0.2, 0.3, 5);


        sleep(1000);

        System.out.println("ValleyX turning to 90 heading");
        rotateToHeading.DoIt(90);
        //rotatePrecise.RotatePrecise(90, 2, 0.2, 0.3, 5);


        sleep(1000);

        System.out.println("ValleyX turning to 270 heading");
        rotateToHeading.DoIt(270);
        //rotatePrecise.RotatePrecise(90, 2, 0.2, 0.3, 5);


        sleep(1000);

        System.out.println("ValleyX turning to 0 heading");
        rotateToHeading.DoIt(0.1);
        //rotatePrecise.RotatePrecise(90, 2, 0.2, 0.3, 5);
        sleep(1000);

        System.out.println("ValleyX turning to 45 heading");
        rotateToHeading.DoIt(45);
        sleep(1000);

        System.out.println("ValleyX turning to 315 heading");
        rotateToHeading.DoIt(315);
        sleep(1000);
        System.out.println("ValleyX turning to 315 heading");
        rotateToHeading.DoIt(0.1);
        sleep(1000);

    }
}
