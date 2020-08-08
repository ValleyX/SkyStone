package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

public class GamePadServoStrafeDrive {
/*
    @TeleOp(name = "test: Test Gamepad mechum", group = "Test")
    @Disabled
    public class GamePadTestNoobCoderzMechum extends LinearOpMode {
        //DcMotor LeftFrontMotor;
        //DcMotor RightFrontMotor;
        //DcMotor LeftBackMotor;
        //DcMotor RightBackMotor;

        public static final double Latch_UP_POSITION = -0.25;
        public static final double Latch_DOWN_POSITION = 0.70;

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

            System.out.println("ValleyX waiting for start");
            waitForStart();
            System.out.println("ValleyX starting");

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

                if ((RightButton > 0) && (LeftButton == 0)) {
                    robot.LeftFrontDrive.setPower(-RightButton);
                    robot.RightFrontDrive.setPower(RightButton);
                    robot.LeftBackDrive.setPower(RightButton);
                    robot.RightBackDrive.setPower(-RightButton);
                }

                if ((LeftButton > 0) && (RightButton == 0)) {
                    robot.LeftFrontDrive.setPower(LeftButton);
                    robot.RightFrontDrive.setPower(-LeftButton);
                    robot.LeftBackDrive.setPower(-LeftButton);
                    robot.RightBackDrive.setPower(LeftButton);

                    while (opModeIsActive()) {
                        boolean DpadBackLatchLeft = gamepad1.dpad_left;
                        boolean DpadBackLatchRight = gamepad1.dpad_right;

                        telemetry.addData("DpadBackLatchLeft=%d", DpadBackLatchLeft);
                        telemetry.addData("DpadBackLatchRight=%d", DpadBackLatchRight);

                        if ((DpadBackLatchUp == true) && (DpadBackLatchDown == false)) {
                            robot.BackRightHook.setPosition(Latch_UP_POSITION);
                            robot.BackLeftHook.setPosition(Latch_DOWN_POSITION);
                            System.out.println("ValleyX LatchLeftPress");
                        }

                        if ((DpadBackLatchDown == true) && (DpadBackLatchUp == false)) {
                            robot.BackRightHook.setPosition(Latch_DOWN_POSITION);
                            robot.BackLeftHook.setPosition(Latch_UP_POSITION);
                            System.out.println("ValleyX Latch Right Press");
                        }
                    }

                }


            }
        }
    }
*/
}
