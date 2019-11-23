package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TwistyClaw
{
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private double ServoNum;

    /* Constructor setup all class variables here */
    public TwistyClaw(RobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
    }

    public void StartAction(double degrees,
                            double timeoutS,
                            boolean waiting) //are we returned only when complete?
    {
        waiting_ = waiting;

        ServoNum = degrees / 270; // servo turns 270 degrees from 0 to 1

        if (robot_.OpMode_.opModeIsActive())
        {
            robot_.twistyClaw.setPosition(ServoNum);
        }

        if (waiting_)
        {
            //then spin here making sure opmode is active, there is available time, action is still running
            while (robot_.OpMode_.opModeIsActive() &&
                    (runtime_.seconds() < timeoutS) &&
                    !IsActionDone())
            {
                // Display it for the driver.
                robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                        ServoNum);
                robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot_.twistyClaw.getPosition());
                robot_.OpMode_.telemetry.update();
                robot_.OpMode_.idle();
            }
            StopAction();
        }
    }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
        return ((ServoNum+0.1) > robot_.twistyClaw.getPosition()) &&
                (robot_.twistyClaw.getPosition() > (ServoNum-0.1));
    }

    //stop the motors
    public void StopAction()
    {
        waiting_ = false;
    }
}
