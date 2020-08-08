package org.firstinspires.ftc.teamcode.Team12841;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

@TeleOp(name="Driver Control", group="Test")

public class DriverControl extends LinearOpMode {
    //DcMotor LeftFrontMotor;
    //DcMotor RightFrontMotor;
    //DcMotor LeftBackMotor;
    //DcMotor RightBackMotor;
    public final double Latch_UP_POSITION = -0.25;
    public final double Latch_DOWN_POSITION = 0.70;

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
            double joystickLeftY = gamepad1.left_stick_y;
            double joystickRightY = gamepad1.right_stick_y;

            double RightButton = gamepad1.right_trigger;
            double LeftButton = gamepad1.left_trigger;

            boolean DpadHooksUp = gamepad1.dpad_up;
            boolean DpadHooksDown = gamepad1.dpad_down;

            telemetry.addData("ValleyX joystickLeftY=%d", joystickLeftY);
            telemetry.addData("ValleyX joystickRightY=%d", joystickRightY);

            if ((RightButton == 0) && (LeftButton == 0)) {
                robot.LeftFrontDrive.setPower(-joystickRightY);
                robot.RightFrontDrive.setPower(-joystickLeftY);
                robot.LeftBackDrive.setPower(-joystickRightY);
                robot.RightBackDrive.setPower(-joystickLeftY);

            }

            telemetry.addData("ValleyX right_trigger=%d", RightButton);
            telemetry.addData("ValleyX left_trigger=%d", LeftButton);
            telemetry.update();

            if ((RightButton >0) && (LeftButton == 0)){
                robot.LeftFrontDrive.setPower(RightButton);
                robot.RightFrontDrive.setPower(-RightButton);
                robot.LeftBackDrive.setPower(-RightButton);
                robot.RightBackDrive.setPower(RightButton);
            }

            if ((LeftButton >0) && (RightButton == 0)){
                robot.LeftFrontDrive.setPower(-LeftButton);
                robot.RightFrontDrive.setPower(LeftButton);
                robot.LeftBackDrive.setPower(LeftButton);
                robot.RightBackDrive.setPower(-LeftButton);

            }


            telemetry.addData("ValleyX DpadBackRightHook=%d", DpadHooksUp);
            telemetry.addData("ValleyX DpadBackLeftHook=%d", DpadHooksDown);



            if ((DpadHooksUp == true) && (DpadHooksDown == false)) {
                robot.BackRightHook.setPosition(Latch_UP_POSITION);
                robot.BackLeftHook.setPosition(Latch_DOWN_POSITION);
                System.out.println("ValleyX LatchLeftPress");
            }

            if ((DpadHooksDown == true) && (DpadHooksUp == false)) {
                robot.BackRightHook.setPosition(Latch_DOWN_POSITION);
                robot.BackLeftHook.setPosition(Latch_UP_POSITION);
                System.out.println("ValleyX Latch Right Press");
            }

        }
    }
}