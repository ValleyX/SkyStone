package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.VuforiaPosition;

@TeleOp(name="Test: TestVuforiaPosition", group="Test")
@Disabled

public class TestVuforiaPosition extends LinearOpMode
{
    public void runOpMode() // testcall()
    {
        double XY[];

        TestRobotHardware robot = new TestRobotHardware(hardwareMap, this);
        VuforiaPosition vuforiaPosition = new VuforiaPosition(robot);

        waitForStart();

        XY = vuforiaPosition.GetVuforiaPosition();
        System.out.printf("ValleyX x = %f, y = %f, target visible = %f\n", XY[0], XY[1], XY[2]);
    }
}
