package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTo
{
    private RobotHardware robot_;
    private EncoderDrive encoderDrive_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    public DriveTo(RobotHardware robot, EncoderDrive encoderDrive)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        encoderDrive_ = encoderDrive;
    }

    public double StartAction(double speed, double inchesTo, double timeoutS, boolean waiting) //are we returned only when complete?
    {
        waiting_ = waiting;

        double drivingDistance = 0;

        //robot_.OpMode_.sleep(1000);
        double DistanceSensor = robot_.sensorRange.getDistance(DistanceUnit.INCH);

        robot_.OpMode_.telemetry.addData("distance sensor", DistanceSensor);
        robot_.OpMode_.telemetry.update();
        System.out.println("ValleyX distance sensor " + DistanceSensor);
        System.out.println("ValleyX inchesTo " + inchesTo);

        if (robot_.OpMode_.opModeIsActive() && (DistanceSensor > inchesTo))
        {
            drivingDistance = DistanceSensor - (inchesTo);

            System.out.println("ValleyX distance sensor " + DistanceSensor);
            System.out.println("ValleyX driving Distance "+ drivingDistance);

            encoderDrive_.StartAction(speed, -drivingDistance, -drivingDistance, 5, true);

            robot_.OpMode_.telemetry.addData("distance moved ", drivingDistance);
        }
        return drivingDistance;
    }
}
