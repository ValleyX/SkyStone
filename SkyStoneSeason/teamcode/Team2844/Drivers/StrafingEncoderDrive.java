
package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StrafingEncoderDrive
{
    private TestRobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public StrafingEncoderDrive(TestRobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
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
    public void Strafe(double speed,
                              double xInches,
                              double timeoutS,
                              boolean waiting) //are we returned only when complete?
    {
        waiting_ = waiting;

        double leftFrontRightBackInches = xInches; //right=positive, left=negative
        double rightFrontLeftBackInches = -xInches; //right=negative, left=positive

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive())
        {
            //setup encoders and motors for this use

            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders ");
            robot_.OpMode_.telemetry.update();

            robot_.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot_.leftFrontDrive.getCurrentPosition(),
                    robot_.leftBackDrive.getCurrentPosition(),
                    robot_.rightFrontDrive.getCurrentPosition(),
                    robot_.rightBackDrive.getCurrentPosition());

            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot_.leftFrontDrive.getCurrentPosition() + (int) (leftFrontRightBackInches * robot_.COUNTS_PER_INCH_STRAFE);
            newLeftBackTarget = robot_.leftBackDrive.getCurrentPosition() + (int) (rightFrontLeftBackInches * robot_.COUNTS_PER_INCH_STRAFE);
            newRightFrontTarget = robot_.rightFrontDrive.getCurrentPosition() + (int) (rightFrontLeftBackInches * robot_.COUNTS_PER_INCH_STRAFE);
            newRightBackTarget = robot_.rightBackDrive.getCurrentPosition() + (int) (leftFrontRightBackInches * robot_.COUNTS_PER_INCH_STRAFE);
            robot_.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot_.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot_.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot_.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot_.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.leftFrontDrive.setPower(Math.abs(speed));
            robot_.leftBackDrive.setPower(Math.abs(speed));
            robot_.rightFrontDrive.setPower(Math.abs(speed));
            robot_.rightBackDrive.setPower(Math.abs(speed));

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
                            newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                    robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot_.leftFrontDrive.getCurrentPosition(),
                            robot_.leftBackDrive.getCurrentPosition(),
                            robot_.rightFrontDrive.getCurrentPosition(),
                            robot_.rightBackDrive.getCurrentPosition());
                    robot_.OpMode_.telemetry.update();
                    robot_.OpMode_.idle();
                }
                StopAction();
            }
        }
    }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
        return  !robot_.leftFrontDrive.isBusy() &&
                !robot_.leftBackDrive.isBusy() &&
                !robot_.rightFrontDrive.isBusy() &&
                !robot_.rightBackDrive.isBusy();
    }

    //stop the motors
    public void StopAction()
    {
        // Stop all motion; 
        robot_.leftFrontDrive.setPower(0);
        robot_.leftBackDrive.setPower(0);
        robot_.rightFrontDrive.setPower(0);
        robot_.rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot_.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }

}
