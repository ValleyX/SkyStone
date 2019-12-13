package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.GoToPosition;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.TestRobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.VuforiaPosition;

@Autonomous(name="Red SkyStone Autonomous", group="Autonomous")
//@Disabled
public class RedSkyStoneAutonomous extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);
        VuforiaPosition vuforiaPosition = new VuforiaPosition(robot);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        GoToPosition GoToPosition = new GoToPosition(robot, rotatePrecise, rotateToHeading, vuforiaPosition, encoderDrive);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);

        waitForStart();

        /*
        System.out.println("ValleyX driving to stones");
        encoderDrive.StartAction(0.6, 30, 30, 5, true);
        sleep(1000);

        // find SkyStone and grab

        System.out.println("ValleyX going to foundation");
        GoToPosition.GoToPosition(23, 35);
        sleep(1000);

        // place stone

        System.out.println("ValleyX going back to stones");
        encoderDrive.StartAction(0.6, -6, -6, 5, true);
        GoToPosition.GoToPosition(-48,35);
         */

        System.out.println("ValleyX driving backwards 28 inches");
        encoderDrive.StartAction(0.6, -28, -28, 5, true);

        System.out.println("ValleyX strafing left 10 inches");
        Strafing.Strafe(0.6, -10, 5, true);
    }
}
