package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.FlippyDriver;

public class PlatformLaser
{
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private double ServoNum;
    private FlippyDriver flippy_;

    //Constructor setup all class variables here
    public PlatformLaser(RobotHardware robot, FlippyDriver flippy)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        flippy_ = flippy;
    }

    public void StartAction(double laser,
                            double timeoutS,
                            boolean waiting)//are we returned only when complete?
    {
        if (robot_.bucketLazery.getDistance(DistanceUnit.INCH) < 0.5)
        {
            robot_.platformy.setPosition(0.57);

            //grab block w arm
        }
        else
        {
            robot_.platformy.setPosition(0.26);

            //flippy_.GoToPosition(0.2, 0.6);
            robot_.flippy.setPower(0.15);
        }
    }
}
