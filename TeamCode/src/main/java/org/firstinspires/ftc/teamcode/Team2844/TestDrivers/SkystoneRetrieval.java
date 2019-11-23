package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.DriverControls;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.ColorDriver;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.DriveTo;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;


@Autonomous(name = "Test: SkystoneRetrieval", group ="Test")
public class SkystoneRetrieval extends LinearOpMode {
    ColorDriver colorDriver;
    EncoderDrive encoderDrive;
    TestRobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TestRobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);
        colorDriver = new ColorDriver(robot);

        //code for test bot only
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);
        DriveTo driveTo = new DriveTo(robot, encoderDrive);

        double distanceMoved;
        
        waitForStart();
        while (opModeIsActive()){
            encoderDrive.StartAction(.5, -28, -28, 5, true);

            //test code
            rotateToHeading.DoIt(0);
            driveTo.StartAction(0.5, 1.5, 5, true);
            //test code


            sleep(1000);
            int skystone = 0;
            while (colorDriver.isSeen() && colorDriver.isYellow() && skystone < 2){
                Strafing.Strafe(.5, -10, 5, true);

                //test code
                rotateToHeading.DoIt(0);
                driveTo.StartAction(0.5, 1.5, 5, true);
                //test code

                telemetry.addData("Skystonenotfound", skystone);
                telemetry.update();
                skystone += 1;
                sleep(1000);
            }
            if (colorDriver.isSeen() && !colorDriver.isYellow()){
                telemetry.addData("Skystoneisfound", skystone);
                telemetry.update();
            } else{
                telemetry.addData("NOTHING", skystone);
                telemetry.update();
            }
            sleep(5000);
            break;
        }

        encoderDrive.StartAction(1.0, 5, 5, 5, true);
        rotatePrecise.RotatePrecise(-90, 1, 0.2, 0.3, 5);

        encoderDrive.StartAction(1.0, -50, -50, 5, true);
        distanceMoved = driveTo.StartAction(1.0, 2, 5, true);

        encoderDrive.StartAction(1.0, 50+distanceMoved, 50+distanceMoved, 5, true);
        rotatePrecise.RotatePrecise(90, 1, 0.2, 0.3, 5);

        driveTo.StartAction(1.0, 2, 5, true);

    }
}
