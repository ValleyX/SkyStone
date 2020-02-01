package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

public class FlippyDriver {
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private boolean inHold_;

    /* Constructor setup all class variables here */
    public FlippyDriver(RobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        inHold_ = false;
    }

    final double inside90 = 0.25;
    final double top = 1.0;
    final double outside90 = 2.0;

    final double maxHoldPower = 0.6;

    public void GoToPosition( double position, double power, double holdPower)
    {
        double timeoutS = 4.0;
        double initialPower = 0.7;
        //double holdPower = .2;
        runtime_.reset();

        double distanceRatio = Math.abs(robot_.flippyPot.getVoltage() - position)/ position;
        double appliedPower = power * distanceRatio;

        holdPower = Math.abs((robot_.flippyPot.getVoltage() - top)) / top;
        appliedPower = (appliedPower + holdPower) / 2;

        robot_.OpMode_.telemetry.addData("requested position ", position);
        robot_.OpMode_.telemetry.addData("requested power ", power);
        robot_.OpMode_.telemetry.addData("Flippy position ", robot_.flippyPot.getVoltage());
        robot_.OpMode_.telemetry.addData("distanceRatio ", distanceRatio);

        if (robot_.flippyPot.getVoltage() < position) {
            robot_.flippy.setPower(appliedPower);
            robot_.OpMode_.telemetry.addData("Applied Power ", appliedPower);
        }
        else {
            robot_.flippy.setPower(-appliedPower);
            robot_.OpMode_.telemetry.addData("Applied Power ", -appliedPower);
        }
/*
            //if (inHold_) {
           //     if (robot_.flippy.getPower() < holdPower) {
             //       robot_.flippy.setPower(initialPower);
            //        robot_.OpMode_.sleep(1);
            //    } else {
            //        robot_.flippy.setPower(appliedPower);
           //         robot_.OpMode_.telemetry.addData("Applied Power ", appliedPower);
//
           //     }
           // }
        }
        else {
            if (inHold_) {
                if (robot_.flippy.getPower() < holdPower) {
                    robot_.flippy.setPower(initialPower);
                    robot_.OpMode_.sleep(1);
                } else {
                    robot_.flippy.setPower(appliedPower);
                    robot_.OpMode_.telemetry.addData("Applied Power ", appliedPower);

                }
            }
        }
           // while ((robot_.flippyPot.getVoltage() < position) && (runtime_.seconds() < timeoutS) && robot_.OpMode_.opModeIsActive())
            //{
                //System.out.println("ValleyX In Less goToPosition pot voltage " + robot_.flippyPot.getVoltage() + " position " + position );

                robot_.OpMode_.idle();
         //   }

            //robot_.flippy.setPower(0.0);
            //holdPower = Math.abs((robot_.flippyPot.getVoltage() - top)) / top;
            //holdPower = holdPower * maxHoldPower;
           // robot_.flippy.setPower(holdPower);
            //robot_.OpMode_.telemetry.addData("holdPower ", holdPower);
        }
        else
        {
            if (Math.abs(robot_.flippy.getPower()) < Math.abs(holdPower))
            {
                robot_.flippy.setPower(-initialPower);

                robot_.OpMode_.sleep(1);
            }
            else
            {
                robot_.flippy.setPower(-appliedPower);
                robot_.OpMode_.telemetry.addData("Applied Power ", -appliedPower);

            }
           // while ((robot_.flippyPot.getVoltage() > position) && (runtime_.seconds() < timeoutS) && robot_.OpMode_.opModeIsActive())
         //   {
                //System.out.println("ValleyX In more goToPosition pot voltage " + robot_.flippyPot.getVoltage() + " position " + position );

                robot_.OpMode_.idle();
          //  }
            //robot_.flippy.setPower(0.0);
            //holdPower = Math.abs((robot_.flippyPot.getVoltage() - top)) / top;
            //holdPower = holdPower * maxHoldPower;
           // robot_.flippy.setPower(holdPower);
           // robot_.OpMode_.telemetry.addData("holdPower ", holdPower);
        }
        if (!robot_.OpMode_.opModeIsActive())
        {
            return;
        }
*/
    }

}


