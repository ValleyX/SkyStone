package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@TeleOp(name = "DistanceAllignment", group = "Sensor")

public class DistanceAllignment extends LinearOpMode
{

    private RobotHardware robot_;

    private Rev2mDistanceSensor distancey1;
    private Rev2mDistanceSensor distancey2;
    RevBlinkinLedDriver blinky;

    @Override
    public void runOpMode () throws InterruptedException  {
        distancey1 = hardwareMap.get(Rev2mDistanceSensor.class, "Distance_Sensory1");
        distancey2 = hardwareMap.get(Rev2mDistanceSensor.class, "Distance_Sensory2");
        blinky = hardwareMap.get(RevBlinkinLedDriver.class, "Blinker_Revy");

        while (opModeIsActive()) {
            if (distancey1.getDistance(DistanceUnit.INCH) < 5 &&
                    distancey2.getDistance(DistanceUnit.INCH) > 5){
                blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (distancey2.getDistance(DistanceUnit.INCH) < 5 &&
                    distancey1.getDistance(DistanceUnit.INCH) > 5){
                blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            } else if (distancey1.getDistance(DistanceUnit.INCH) < 5 &&
                    distancey2.getDistance(DistanceUnit.INCH) < 5){
                blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            telemetry.addData("distance", String.format("% .01f in", distancey1.getDistance(DistanceUnit.INCH)));
            telemetry.addData("distance2", String.format("%.01f in", distancey2.getDistance(DistanceUnit.INCH)));

            telemetry.update();
        }
    }
}
