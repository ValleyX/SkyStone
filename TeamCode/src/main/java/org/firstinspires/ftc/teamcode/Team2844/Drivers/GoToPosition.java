package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.TestDrivers.ConceptVuforiaSkyStoneNavigation;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class GoToPosition
{
    //Variables we understand
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private RotatePrecise rotatePrecise_;
    private RotateToHeading rotateToHeading_;
    private VuforiaPosition vuforiaPosition_;
    private EncoderDrive encoderDrive_;

    /* Constructor setup all class variables here */
    public GoToPosition(RobotHardware robot, RotatePrecise rotatePrecise, RotateToHeading rotateToHeading, VuforiaPosition vuforiaPosition, EncoderDrive encoderDrive)
    {
        //VARIABLES WE UNDERSTAND
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        rotatePrecise_ = rotatePrecise;
        rotateToHeading_ = rotateToHeading;
        vuforiaPosition_ = vuforiaPosition;
        encoderDrive_ = encoderDrive;

        /* ---new remapping code --*/
        //swapping y & z axis due to vertical mounting of rev expansion board
        //byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_CONFIG_BYTE = 0x18; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);

        robot_.OpMode_.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE);

        //Write to the AXIS_MAP_SIGN register
        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE);

        //Need to change back into the IMU mode to use the gyro
        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        robot_.OpMode_.sleep(100); //Changing modes again requires a delay
        /* ---new remapping code ---*/

        BNO055IMU.Parameters imu_parameters = new BNO055IMU.Parameters(); // inertial motion unit

        imu_parameters.mode = BNO055IMU.SensorMode.IMU;
        imu_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_parameters.loggingEnabled = false;

        robot_.imu.initialize(imu_parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!robot_.OpMode_.isStopRequested() && !robot_.imu.isGyroCalibrated()) {
            robot_.OpMode_.sleep(50);
            robot.OpMode_.idle();
        }
        if (!robot_.imu.isGyroCalibrated())
        {
            System.out.println("ValleyX: Gyro not calibrated");
        }

        System.out.println("ValleyX: imu calib status" + robot_.imu.getCalibrationStatus().toString());
        robot_.OpMode_.telemetry.addData("Mode", "calibrated");
        robot_.OpMode_.telemetry.update();
        double gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        robot_.OpMode_.telemetry.addData("imu heading", gyroActual);
        robot_.OpMode_.telemetry.update();
    }

    public void GoToPosition (float XValue, float YValue) {
        double XY[];

        System.out.printf("ValleyX:  GoToPos Requested {X, Y} = %.1f, %.1f\n",
                XValue, YValue);

        XY = vuforiaPosition_.GetVuforiaPosition();
        System.out.printf("ValleyX:  vuforia initial positions {X, Y, Z} = %.1f, %.1f, %.1f\n",
                XY[0] , XY[1], XY[2]);
        robot_.OpMode_.telemetry.addData("vuforia positions", XY);
        robot_.OpMode_.telemetry.update();

        int timer = 0;
        while (timer < 10)
        {
            if (XY[2] == 0)
            {
                System.out.println("ValleyX searching for picture");
                rotatePrecise_.RotatePrecise(90, 2, 0.2, 0.3, 5);
                robot_.OpMode_.sleep(10); //let vuforia settle, maybe make this an idle
                XY = vuforiaPosition_.GetVuforiaPosition();

                //System.out.println("ValleyX vuforia potisions" + XY);
                System.out.printf("ValleyX:  vuforia positions {X, Y, Z} = %.1f, %.1f, %.1f\n",
                        XY[0] , XY[1], XY[2]);
                //robot_.OpMode_.telemetry.addData("vuforia positions", XY);
                //robot_.OpMode_.telemetry.update();
            }
            if (XY[2] == 1)
            {
                System.out.println("ValleyX picture found");
                double YDistance = YValue - XY[1];
                System.out.println("ValleyX y distance " + YDistance);
                robot_.OpMode_.telemetry.addData("y distance", YDistance);
                robot_.OpMode_.telemetry.update();
                if (YDistance < 0) {
                    System.out.println("ValleyX starting turn to 0 heading");
                    rotateToHeading_.DoIt(0);
                    System.out.println("ValleyX finished turn to 0 heading");
                } else {
                    System.out.println("ValleyX starting turn to 180 heading");
                    rotateToHeading_.DoIt(180);
                    System.out.println("ValleyX finished turn to 180 heading");
                }
                System.out.println("ValleyX driving YDistance " + abs(YDistance));
                encoderDrive_.StartAction(0.6, abs(YDistance), abs(YDistance), 5.0, true);

                double XDistance = XValue - XY[0];
                System.out.println("ValleyX x distance " + XDistance);
                robot_.OpMode_.telemetry.addData("x distance", XDistance);
                robot_.OpMode_.telemetry.update();
                if (XDistance < 0) {
                    System.out.println("ValleyX starting turn to 90 heading");
                    rotateToHeading_.DoIt(90);
                    System.out.println("ValleyX turn completed to 90 heading");
                } else {
                    System.out.println("ValleyX starting turn to -90 heading");
                    rotateToHeading_.DoIt(-90);
                    System.out.println("ValleyX turn completed to -90 heading");
                }

                //robot_.OpMode_.telemetry.update();
                //robot_.OpMode_.sleep(5000);
                System.out.println("ValleyX driving XDistance " + abs(XDistance));
                encoderDrive_.StartAction(0.6, abs(XDistance), abs(XDistance), 5.0, true);

                break;
            }

            robot_.OpMode_.sleep(10);
            timer++;
        }
    }
}