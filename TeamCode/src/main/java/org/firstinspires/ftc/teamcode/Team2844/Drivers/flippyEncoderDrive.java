
package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class flippyEncoderDrive
{
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private boolean isRunning_;

    /* Constructor setup all class variables here */
    public flippyEncoderDrive(RobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        isRunning_ = false;

        robot.flippy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flippy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  If waiting is true then StartAction will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     *  If waiting is false, then the action will start, but it is the caller's
     *  responsibility to loop and check the isActionDone() for completion
     *  and to stop the motors when complete
     *  This feature allow the main program to start up multiple robot actions
     *  in parallel in a larger loop checking multiple robots actions for completion
     */
    public void StartAction(double speed,
                            double height,
                            double timeoutS,
                            boolean waiting) //are we returned only when complete?
    {
        waiting_ = waiting;

        int liftHeight;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive())
        {
            //setup encoders and motors for this use

            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders ");
            robot_.OpMode_.telemetry.update();

            robot_.flippy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.flippy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot_.flippy.getCurrentPosition());

            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            liftHeight = robot_.flippy.getCurrentPosition() + (int) (height * robot_.COUNTS_PER_INCH);
            robot_.flippy.setTargetPosition(liftHeight);

            // Turn On RUN_TO_POSITION
            robot_.flippy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.flippy.setPower(Math.abs(speed));

            runtime_.reset();

            System.out.println("ValleyX waiting_ " + waiting_);
            isRunning_ = false;

            if (waiting_)
            {
                //then spin here making sure opmode is active, there is available time, action is still running
                while (robot_.OpMode_.opModeIsActive() &&
                      (runtime_.seconds() < timeoutS) &&
                      !IsActionDone())
                {
                    // Display it for the driver.
                    robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                            liftHeight);
                    robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot_.flippy.getCurrentPosition());
                    robot_.OpMode_.idle();
                }
                StopAction();
            }
            else
            {
                System.out.println("ValleyX Setting isRunning to true");
                isRunning_ = true;
            }
        }
    }

    public boolean IsRunning() { return isRunning_; }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
        if (!robot_.flippy.isBusy())
        {
            isRunning_ = false;
            System.out.println("ValleyX setting isRunning to false");
            return true;
        }
        else
        {
            return false;
        }
    }

    //stop the motors
    public void StopAction()
    {
        // Stop all motion; 
        robot_.flippy.setPower(0);

        // Turn off RUN_TO_POSITION
        System.out.println("ValleyX set RUN_WITHOUT_ENCODER");
        robot_.flippy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }

    public void MoveToEncoderValue
            (
            double speed,
            double posFrom0, //0..1
            double timeoutS,
            boolean waiting)
    {
        robot_.flippy.setPower(0);

        waiting_ = waiting;

        if (posFrom0 > .7)
            return;

        System.out.println("ValleyX current lift position before " + robot_.flippy.getCurrentPosition());
        int position = (int) (posFrom0 * robot_.ONE_MOTOR_COUNT_FLIPPY);

        robot_.flippy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.flippy.setTargetPosition(position);

        // Turn On RUN_TO_POSITION
        robot_.flippy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot_.flippy.setPower(Math.abs(speed));

        runtime_.reset();

        isRunning_ = false;

        if (waiting_)
        {
            //then spin here making sure opmode is active, there is available time, action is still running
            while (robot_.OpMode_.opModeIsActive() &&
                    (runtime_.seconds() < timeoutS) &&
                    !IsActionDone())
            {
                // Display it for the driver.
                /*
                robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                        position);
                robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot_.lift.getCurrentPosition());
                 */
                robot_.OpMode_.idle();
            }
            StopAction();
            robot_.flippy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            System.out.println("ValleyX current flippy position after " + robot_.flippy.getCurrentPosition());
        }
        else
        {
            isRunning_ = true;
        }
    }

    public double CurrentEncoderPosition()
    {
        return (robot_.flippy.getCurrentPosition() / robot_.ONE_MOTOR_COUNT_FLIPPY);
    }

}
