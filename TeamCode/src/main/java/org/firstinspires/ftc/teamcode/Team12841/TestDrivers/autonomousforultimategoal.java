package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

import java.util.Locale;

@Autonomous(name="Test: autonomous for ultimate goal ", group="Test")

public class autonomousforultimategoal extends LinearOpMode {

    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoder = new EncoderDrive(robot);
        double heading;
        waitForStart();

        final double fullturn = 3.14 * 18; //18 inches
        final double halfturn = 3.14 * 9; // 9 inches
        final double quarterturn = 3.14 * 4.5; //4.5 inches



    /*kinda blue box A (right start)
        encoder.StartAction(0.5,78,78,30,true);

        //turn left 90 degress
        robot.leftDrive.setPower(-0.15);
        robot.rightDrive.setPower(0.15);

        //these 2 lines get heading from IMU
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.firstAngle);

        System.out.println("ValleyX: " + heading);
        while (heading <= 89){
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.firstAngle);
            System.out.println("ValleyX: " + heading);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        encoder.StartAction(0.5,22,22,30,true);
        encoder.StartAction(0.5,-22,-22,30,true);
        */

    //kinda blue box B (right start)
        encoder.StartAction(0.5,97,97,30,true);

        robot.leftDrive.setPower(-0.15);
        robot.rightDrive.setPower(0.15);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.firstAngle);

        System.out.println("ValleyX: " + heading);
        while (heading <= 89){
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.firstAngle);
            System.out.println("ValleyX: " + heading);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        sleep(5000);

        robot.leftDrive.setPower(0.15);
        robot.rightDrive.setPower(-0.15);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.firstAngle);

        System.out.println("ValleyX: " + heading);
        while (heading >= 0){
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.firstAngle);
            System.out.println("ValleyX: " + heading);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        encoder.StartAction(0.5,-25,-25,10,true);


    /*kinda blue box C (right start)
        encoder.StartAction(0.5, 123, 123, 30, true);
        robot.leftDrive.setPower(-0.15);
        robot.rightDrive.setPower(0.15);

        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //heading = formatAngle(angles.angleUnit, angles.firstAngle);
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        System.out.println("ValleyX left: " + heading);
        while (heading <= 89){
            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //heading = formatAngle(angles.angleUnit, angles.firstAngle);
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            System.out.println("ValleyX left: " + heading);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        encoder.StartAction(0.5, 25, 25, 30, true);

        encoder.StartAction(0.5,-25,-25,30,true);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        System.out.println("ValleyX : before sleep");
        sleep(3000);
        System.out.println("ValleyX : After sleep");

        robot.leftDrive.setPower(0.15);
        robot.rightDrive.setPower(-0.15);
        System.out.println("ValleyX after power:");
        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //heading = formatAngle(angles.angleUnit, angles.firstAngle);
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        System.out.println("ValleyX right: " + heading);
        while (heading >= 0){
            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //heading = formatAngle(angles.angleUnit, angles.firstAngle);
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            System.out.println("ValleyX right: " + heading);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        encoder.StartAction(0.5,-54,-54,30,true);
*/
    }

    double formatAngle(AngleUnit angleUnit, double angle) {
        double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);
        double normalDegrees = AngleUnit.DEGREES.normalize(degrees);

        return normalDegrees;
        //double heading =  formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
