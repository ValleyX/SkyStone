package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.VuforiaPosition;

@TeleOp(name="Test: TestEncoderDriveHeading", group="Test")
public class TestEncoderDriveHeading  extends LinearOpMode {

    public void runOpMode() // need to test
    {
        RobotHardware robot_ = new RobotHardware(hardwareMap, this);
        EncoderDriveHeading encoderDriveHeading = new EncoderDriveHeading(robot_);
        RotatePrecise rotatePrecise = new RotatePrecise(robot_);
        RotateToHeading rotateToHeading = new RotateToHeading(robot_, rotatePrecise);
        StrafingEncoderDrive strafing = new StrafingEncoderDrive(robot_);
        StrafingEncoderDriveHeading strafeGyro = new StrafingEncoderDriveHeading(robot_);


        /* ---new remapping code --*/
        //swapping y & z axis due to vertical mounting of rev expansion board
        //byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_CONFIG_BYTE = 0x18; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);

        sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE);

        //Write to the AXIS_MAP_SIGN register
        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE);

        //Need to change back into the IMU mode to use the gyro
        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay
        /* ---new remapping code ---*/

        BNO055IMU.Parameters imu_parameters = new BNO055IMU.Parameters(); // inertial motion unit

        imu_parameters.mode = BNO055IMU.SensorMode.IMU;
        imu_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_parameters.loggingEnabled = false;

        robot_.imu.initialize(imu_parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot_.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        if (!robot_.imu.isGyroCalibrated())
        {
            System.out.println("ValleyX: Gyro not calibrated");
        }

        System.out.println("ValleyX: imu calib status" + robot_.imu.getCalibrationStatus().toString());
        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        /*
        encoderDriveHeading.StartAction(0.9, 5, 0, 5, true);

        rotateToHeading.DoIt(-90);
        sleep(1000);

        encoderDriveHeading.StartAction(.9,47.5, -90,5, true);
        sleep(1000);
        encoderDriveHeading.StartAction(.9,-47.5, -90,5, true);
        sleep(1000);
        encoderDriveHeading.StartAction(.9,47.5, -90,5, true);
        sleep(1000);
        encoderDriveHeading.StartAction(.9,-47.5, -90,5, true);
        sleep(1000);
         */

        strafing.Strafe(0.9, 47.5, 5, true);
        //strafeGyro.Strafe(.9,47.5, -90,5,true);
        rotateToHeading.DoIt(0);

        sleep(1000);
        //strafeGyro.Strafe(0.9, -47.5, 90, 5, true);
        strafing.Strafe(0.9, -47.5, 5, true);
        rotateToHeading.DoIt(0);
        //strafeGyro.Strafe(0.6, -47.5, 90, 5, true);
        sleep(1000);
    }

}