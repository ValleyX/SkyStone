package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RotatePrecise
{
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public RotatePrecise(RobotHardware robot)
    {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // inertial motion unit

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
    }


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port

    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",

    // and named "imu".


    public void RotatePrecise(double gyroTarget, double gyroRange, double minSpeed, double addSpeed, int timesCorrect)
    {
        double turnPower = 0;
        double gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        gyroTarget += gyroActual + 360.0;
        gyroTarget %= 360;
        int correctCount = 0;

        while ((correctCount < timesCorrect) && robot_.OpMode_.opModeIsActive())

        {
            gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360


            if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
            if (Math.abs(delta) > gyroRange)
            { //checks if delta is out of range
                correctCount = 0;
                double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
                if (Math.abs(gyroMod) > 1.0)
                    gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
                turnPower = minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod;
            }
            else {
                correctCount++;
                turnPower = 0;
            }
            double  leftPower, rightPower;
            if (gyroTarget < 0)
            {   // turn right
                leftPower = -turnPower;
                rightPower = turnPower;
            }
            else
            {   // turn left
                leftPower = turnPower;
                rightPower = -turnPower;
            }

            // set power to rotate.
            robot_.leftFrontDrive.setPower(leftPower);
            robot_.leftBackDrive.setPower(leftPower);
            robot_.rightFrontDrive.setPower(rightPower);
            robot_.rightBackDrive.setPower(rightPower);
        }
        //   return this.correctCount;
    }
}
