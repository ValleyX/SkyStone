package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

@TeleOp(name="test: Test Gamepad mechum", group="Test")

public class GamePadTestNoobCoderzMechum extends LinearOpMode {
    //DcMotor LeftFrontMotor;
    //DcMotor RightFrontMotor;
    //DcMotor LeftBackMotor;
    //DcMotor RightBackMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, this);
       // LeftFrontMotor = hardwareMap.get(DcMotor.class, "lfmotor");
        //RightFrontMotor = hardwareMap.get(DcMotor.class, "rfmotor");
       // LeftBackMotor = hardwareMap.get(DcMotor.class, "lbmotor");
       // RightBackMotor = hardwareMap.get(DcMotor.class, "rbmotor");
       // LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // TODO determine which motor should be reversed
        //RightFrontMotor.setDirection(DcMotor.Direction.FORWARD); // TODO determine which motor should be reversed
        //LeftBackMotor.setDirection(DcMotor.Direction.FORWARD); // TODO determine which motor should be reversed
        //RightBackMotor.setDirection(DcMotor.Direction.REVERSE); // TODO determine which motor should be reversed


        waitForStart();

        while (opModeIsActive()) {
            double joystickLeftX = gamepad1.left_stick_y;
            double joystickRightY = gamepad1.right_stick_y;

            double RightButton = gamepad1.right_trigger;
            double LeftButton = gamepad1.left_trigger;

            telemetry.addData("joystickLeftX=%d", joystickLeftX);
            telemetry.addData("joystickRightY=%d", joystickRightY);

            if ((RightButton == 0) && (LeftButton == 0)) {
                robot.LeftFrontDrive.setPower(joystickRightY);
                robot.RightFrontDrive.setPower(joystickRightY);
                robot.LeftBackDrive.setPower(joystickLeftX);
                robot.RightBackDrive.setPower(joystickLeftX);

            }

            telemetry.addData("right_trigger=%d", RightButton);
            telemetry.addData("left_trigger=%d", LeftButton);
            telemetry.update();

            if ((RightButton >0) && (LeftButton == 0)){
                robot.LeftFrontDrive.setPower(-RightButton);
                robot.RightFrontDrive.setPower(RightButton);
                robot.LeftBackDrive.setPower(RightButton);
                robot.RightBackDrive.setPower(-RightButton);
            }

            if ((LeftButton >0) && (RightButton == 0)){
                robot.LeftFrontDrive.setPower(LeftButton);
                robot.RightFrontDrive.setPower(-LeftButton);
                robot.LeftBackDrive.setPower(-LeftButton);
                robot.RightBackDrive.setPower(LeftButton);

            }


        }
    }
}