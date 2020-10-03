package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@TeleOp(name = "RPSTest")

public class RPSTest extends LinearOpMode {
    //@Override

    //private RobotHardware robot_;

    public void runOpMode() {

        RobotHardware robot_ = new RobotHardware(hardwareMap, this);

        waitForStart();

        robot_.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot_.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.leftDrive.setPower(0.7);
        ElapsedTime runtime_ = new ElapsedTime();

        while (opModeIsActive())
        {
            //double rightStickY = gamepad1.right_stick_y;

            double wheelPosition1 = robot_.leftDrive.getCurrentPosition();

            runtime_.reset();
            while(runtime_.seconds() < 1) {
                robot_.rightDrive.setPower(gamepad1.right_stick_y);
            }

            //robot_.OpMode_.sleep(1000);
            double wheelPosition2 = robot_.leftDrive.getCurrentPosition();

            double ENCODER_TICKS_PER_SECOND = wheelPosition2 - wheelPosition1;
            double REVOLUTIONS_PER_SECOND = ENCODER_TICKS_PER_SECOND / robot_.ONE_MOTOR_COUNT;

            robot_.OpMode_.telemetry.addData("RPS: ", REVOLUTIONS_PER_SECOND);
            robot_.OpMode_.telemetry.update();

           // robot_.rightDrive.setPower(rightStickY);
        }
    }
}
