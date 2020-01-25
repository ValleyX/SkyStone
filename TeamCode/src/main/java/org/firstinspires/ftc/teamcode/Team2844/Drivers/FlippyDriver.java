package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FlippyDriver {
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public FlippyDriver(RobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
    }

    public void GoToPosition( double position, double power)
    {
        double timeoutS = 4.0;
        runtime_.reset();
        if (robot_.flippyPot.getVoltage() < position)
        {

            robot_.flippy.setPower(power);
            while ((robot_.flippyPot.getVoltage() < position) && (runtime_.seconds() < timeoutS) && robot_.OpMode_.opModeIsActive())
            {
                //System.out.println("ValleyX In Less goToPosition pot voltage " + robot_.flippyPot.getVoltage() + " position " + position );

                robot_.OpMode_.idle();
            }
            robot_.flippy.setPower(0.0);

        }
        else
        {
            robot_.flippy.setPower(-power);
            while ((robot_.flippyPot.getVoltage() > position) && (runtime_.seconds() < timeoutS) && robot_.OpMode_.opModeIsActive())
            {
                //System.out.println("ValleyX In more goToPosition pot voltage " + robot_.flippyPot.getVoltage() + " position " + position );

                robot_.OpMode_.idle();
            }
            robot_.flippy.setPower(0.0);
        }
        if (!robot_.OpMode_.opModeIsActive())
        {
            return;
        }

    }

}


