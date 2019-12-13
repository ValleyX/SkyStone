package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team2844.DriverControls;
//import org.firstinspires.ftc.teamcode.Team2844.Drivers.ColorDriver;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.DriveTo;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;


@Autonomous(name = "Test: SkystoneRetrieval", group ="Test")
@Disabled
public class SkystoneRetrieval extends LinearOpMode {
    //ColorDriver colorDriver;
    EncoderDrive encoderDrive;
    RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);
        //colorDriver = new ColorDriver(robot);

        //code for test bot only
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);
        DriveTo driveTo = new DriveTo(robot, encoderDrive);

        double distanceMoved;
        int skystone = 0;

        waitForStart();
        while (opModeIsActive()) {
            encoderDrive.StartAction(.5, -28, -28, 5, true);

            //test code
            rotateToHeading.DoIt(0);
            driveTo.StartAction(0.5, 1.5, 5, true);
            //test code

/*
            sleep(1000);
            skystone = 0;
            while (colorDriver.isSeen() && colorDriver.isYellow() && skystone < 2)
            {
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
            sleep(100);
            break;
        }
*/
            double stoneSize = 8;
            double fixedAmount = 50;
            double toFoundation = fixedAmount + (skystone * stoneSize);
            double fromFoundation = toFoundation + (3 * stoneSize);


            // back up from stones to turn
            encoderDrive.StartAction(1.0, 5, 5, 5, true);
            //rotatePrecise.RotatePrecise(-90, 2, 0.2, 0.3, 5);
            rotateToHeading.DoIt(-90);

            // drive to foundation and get 2 inches away
            encoderDrive.StartAction(1.0, -toFoundation, -toFoundation, 5, true);
            sleep(1000);
            int count = 0;
            while (robot.sensorRange.getDistance(DistanceUnit.INCH) > 10) {
                encoderDrive.StartAction(0.6, -8, -8, 5, true);
                count += 1;
            }
            distanceMoved = driveTo.StartAction(0.5, 2, 5, true);
            System.out.println("ValleyX Distance Moved " + distanceMoved);


            // drive back from foundation and turn towards stones
            encoderDrive.StartAction(1.0, fromFoundation + distanceMoved + (count * 8), fromFoundation + distanceMoved + (count * 8), 5, true);
            rotateToHeading.DoIt(0);
            driveTo.StartAction(1.0, 2, 5, true);
            // back up from stones to turn (again)
            encoderDrive.StartAction(1.0, 5, 5, 5, true);
            //rotatePrecise.RotatePrecise(-90, 2, 0.2, 0.3, 5);
            rotateToHeading.DoIt(-90);

            // drive to foundation and get 2 inches away (again)
            encoderDrive.StartAction(1.0, -fromFoundation, -fromFoundation, 5, true);
            count = 0;
            while (robot.sensorRange.getDistance(DistanceUnit.INCH) > 10) {
                encoderDrive.StartAction(0.6, -8, -8, 5, true);
                count += 1;
            }
            driveTo.StartAction(1.0, 2, 5, true);
            // park on the tape
            encoderDrive.StartAction(0.6, 30 + (count * 8), 30 + (count * 8), 5, true);
        }
    }
}
