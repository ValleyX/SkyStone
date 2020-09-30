package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

@Autonomous(name="Test: autonomous for ultimate goal ", group="Test")

public class autonomousforultimategoal extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoder = new EncoderDrive(robot);
        waitForStart();

    final double fullturn = 3.14 * 18; //18 inches
    final double halfturn = 3.14 * 9; // 9 inches
    final double quarterturn = 3.14 * 4.5; //4.5 inches

    //kinda blue box A (right start)
        //encoder.StartAction(1,78,78,30,true);
        //encoder.StartAction(0.3,-quarterturn,quarterturn,5,true);
        //encoder.StartAction(1,26,26,30,true);
    //kinda blue box B (right start)
        //encoder.StartAction(1,97,97,30,true);
        //encoder.StartAction(1, -quarterturn, quarterturn, 5, true);
        //encoder.StartAction(1,3, 3,5, true);
    //kinda blue box C (right start)
        encoder.StartAction(1,123,123, 30,true);
        encoder.StartAction(1,-quarterturn,quarterturn,5,true);


    }
}
