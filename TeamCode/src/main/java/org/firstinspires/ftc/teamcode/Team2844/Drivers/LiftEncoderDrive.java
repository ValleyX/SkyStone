
package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftEncoderDrive
{
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public LiftEncoderDrive(RobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            robot_.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot_.lift.getCurrentPosition());

            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            liftHeight = robot_.lift.getCurrentPosition() + (int) (height * robot_.COUNTS_PER_INCH);
            robot_.lift.setTargetPosition(liftHeight);

            // Turn On RUN_TO_POSITION
            robot_.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.lift.setPower(Math.abs(speed));

            runtime_.reset();

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
                            robot_.lift.getCurrentPosition());
                    robot_.OpMode_.idle();
                }
                StopAction();
            }
        }
    }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
        return !robot_.lift.isBusy();
    }

    //stop the motors
    public void StopAction()
    {
        // Stop all motion; 
        robot_.lift.setPower(0);

        // Turn off RUN_TO_POSITION
        System.out.println("ValleyX set RUN_WITHOUT_ENCODER");
        robot_.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }

    public void MoveToEncoderValue
            (
            double speed,
            int inchesFromBottom,
            double timeoutS,
            boolean waiting)
    {
        robot_.lift.setPower(0);

        waiting_ = waiting;

        System.out.println("ValleyX current lift position before " + robot_.lift.getCurrentPosition());
        int position = (int) (inchesFromBottom * robot_.COUNTS_PER_INCH_LIFT);

        robot_.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.lift.setTargetPosition(position);

        // Turn On RUN_TO_POSITION
        robot_.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot_.lift.setPower(Math.abs(speed));

        runtime_.reset();

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
            robot_.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            System.out.println("ValleyX current lift position after " + robot_.lift.getCurrentPosition());
        }
    }
    public double CurrentEncoderPosition()
    {
        return (robot_.lift.getCurrentPosition() / robot_.COUNTS_PER_INCH_LIFT);
    }

}
