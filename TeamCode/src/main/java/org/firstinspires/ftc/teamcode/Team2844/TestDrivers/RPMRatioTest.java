package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

public class RPMRatioTest
{
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public RPMRatioTest(RobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
    }

    public void StartAction(double speed)
    {
        double wheelPosition1 = robot_.leftDrive.getCurrentPosition();
        robot_.OpMode_.sleep(1000);
        double wheelPosition2 = robot_.leftDrive.getCurrentPosition();

        double ENCODER_TICKS_PER_SECOND = wheelPosition2 - wheelPosition1;
        double REVOLUTIONS_PER_SECOND = ENCODER_TICKS_PER_SECOND / robot_.ONE_MOTOR_COUNT;

        robot_.OpMode_.telemetry.addData("RPS: ", REVOLUTIONS_PER_SECOND);
        robot_.OpMode_.telemetry.update();
    }

}
