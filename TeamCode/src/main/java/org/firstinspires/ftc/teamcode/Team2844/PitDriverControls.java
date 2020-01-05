package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.LiftEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

//import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp: Pit Driver Controls", group="Test")
//@Disabled

public class PitDriverControls extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        LiftEncoderDrive liftEncoderDrive = new LiftEncoderDrive(robot);

        float leftStickY;
        float rightStickY;
        float leftTrigger;
        float rightTrigger;
        float leftTrigger2;
        float rightTrigger2;

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //liftEncoderDrive.MoveToEncoderValue(0.6, 10, 5, true);

        while (opModeIsActive())
        {
            /*
            If the robot turns off with the lift up
            When they turn the robot on again it resets the encoders
            and then wont let them bring the lift down any farther
            this is to let them bring the lift down after a match
             */

            double leftStickY2 = -gamepad2.left_stick_y;
            robot.lift.setPower(leftStickY2);

            if (gamepad2.x)
            {
                robot.swingy.setPosition(0.8);
            }

            telemetry.addData("ValleyX lift encoder", liftEncoderDrive.CurrentEncoderPosition());
            telemetry.update();

            //robot.swingy.setPosition(0.27);

            //robot.twistyClaw.setPosition(0.37);

            //robot.clawy.setPosition(0.67);

            //robot.platformy.setPosition(0.57);

            //robot.twistyClaw.setPosition(0.77);
        }
    }
}
