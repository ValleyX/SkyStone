package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.FlippyDriver;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.flippyEncoderDrive;
import org.firstinspires.ftc.teamcode.dogecvDetectors.BlackRectDetector;
import org.firstinspires.ftc.teamcode.dogecvDetectors.SkyStone2Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//import org.firstinspires.ftc.teamcode.Team2844.Drivers.ColorDriver;


@Autonomous(name = "BlueNonStrafingAutonomous", group ="Test")
@Disabled
public class BlueNonStrafingAutonomous extends LinearOpMode {
    //ColorDriver colorDriver;
    EncoderDrive encoderDrive;
    RobotHardware robot;
    FlippyDriver flippy;
    EncoderDriveHeading encoderDriveHeading;
    //DigitalCamera digitalCamera = new DigitalCamera()
    // SkystoneDetector skystoneDetector;
    SkyStone2Detector skystoneDetector;
    StoneDetector stoneDetector;
    BlackRectDetector blackRectDetector;
    OpenCvCamera phoneCam;
//    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);
        FlippyDriver flippy = new FlippyDriver(robot);
        //colorDriver = new ColorDriver(robot);

        EncoderDriveHeading encoderDriveHeading = new EncoderDriveHeading(robot);
        flippyEncoderDrive flippyEncoderDrive = new flippyEncoderDrive(robot);

        //code for test bot only
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);
        //DriveTo driveTo = new DriveTo(robot, encoderDrive);

        //skystoneDetector = new SkystoneDetector();
        skystoneDetector = new SkyStone2Detector();
        stoneDetector = new StoneDetector();
        blackRectDetector = new BlackRectDetector();

        int leftXLine = 120; // purple
        int rightXLine = 187; // green

        skystoneDetector.SetRequestedYLine(205);
        skystoneDetector.SetRequestedXRightLine(300);
        skystoneDetector.SetRequestedXLeftLine(50);
        skystoneDetector.SetRequestedMidlinesRightLine(leftXLine, rightXLine); // purple, green

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();
//        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(skystoneDetector);
//        webcam.setPipeline(skystoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
         * than 30FPS is not currently supported, although this will likely be addressed in a future
         * release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        //webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        double distanceMoved;
        int skystone = 0;

        //sleep(5000);


        flippy.GoToPosition(0.2, 1,0.6);
        robot.flippy.setPower(0.15);

        robot.rightGrabber.setPosition(0.4);
        robot.leftGrabber.setPosition(0.4);
        final double platformDownPos = 0.27;
        robot.platformy.setPosition(platformDownPos);
        flippyEncoderDrive.MoveToEncoderValue(0.6, 0.0, 5, false);


        final int headingduh = 90;
        final int heading = 45;

        double clawopen = 0.1; //0.0
        double clawclose = 0.45; //0.37 //0.55



        waitForStart();
/*
        flippy.GoToPosition(0.2, 0.6);
        robot.flippy.setPower(0.15);
        robot.clawy.setPosition(0.37);
        sleep(500);
        robot.clawy.setPosition(0.67);
        flippy.GoToPosition(1.5, 0.6);
        //flippy.GoToPosition(2.0, 0.2);
        robot.flippy.setPower(-0.15);


        while(opModeIsActive())
        {
            telemetry.addData("pot position", robot.flippyPot.getVoltage());
            telemetry.update();
        }
*/
        double extra = 0;

        while (opModeIsActive())
        {


                if (skystoneDetector.isDetected()) {
                    //webcam.setPipeline(null);
                    phoneCam.setPipeline(null);
                    telemetry.addData("Skystone found X Y", "%d %d",
                            skystoneDetector.foundRectangle().x, skystoneDetector.foundRectangle().y);
                    int stoneXValue = skystoneDetector.foundRectangle().x;
                    if (leftXLine > stoneXValue) {
                        telemetry.addData("stone found left", stoneXValue);
                        System.out.println("ValleyX stone found left " + stoneXValue);
                        skystone = 0;
                        Strafing.Strafe(0.6, -9, 5, true);
                        robot.platformy.setPosition(platformDownPos);
                        rotateToHeading.DoIt(0);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        extra = 3;
                        encoderDriveHeading.StartAction(0.9, 27, 0, 5, true);
                        encoderDriveHeading.StartAction(0.3, 7+extra, 0, 5, true);
                        sleep(300);
                        //robot.rightIntake.setPower(0.0);
                        //robot.leftIntake.setPower(0.0);
                        break;
                    }
                    if (leftXLine < stoneXValue && stoneXValue < rightXLine) {
                        telemetry.addData("stone found middle", stoneXValue);
                        System.out.println("ValleyX stone found middle " + stoneXValue);
                        skystone = 1;
                        robot.platformy.setPosition(platformDownPos);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDriveHeading.StartAction(0.9, 27, 0, 5, true);
                        encoderDriveHeading.StartAction(0.3, 7, 0, 5, true);
                        sleep(300);
                        //robot.rightIntake.setPower(0.0);
                        //robot.leftIntake.setPower(0.0);
                        break;
                    }
                    if (stoneXValue > rightXLine) {
                        telemetry.addData("stone found right", stoneXValue);
                        System.out.println("ValleyX stone found right " + stoneXValue);
                        skystone = 2;
                        //encoderDriveHeading.StartAction(1.0, 2, 0, 5, true);
                        Strafing.Strafe(0.6, 9, 5, true);
                        rotateToHeading.DoIt(0);
                        robot.platformy.setPosition(platformDownPos);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDriveHeading.StartAction(0.9, 27, 0, 5, true);
                        encoderDriveHeading.StartAction(0.3, 7, 0, 5, true);
                        sleep(300);
                        //robot.rightIntake.setPower(0.0);
                        //robot.leftIntake.setPower(0.0);
                        break;
                    }
                } else {
                    telemetry.addLine("Sky stone not found");
                }
                telemetry.update();
                //webcam.setPipeline(skystoneDetector);
                phoneCam.setPipeline(skystoneDetector);
                idle();
            }

            double FudgeFactor = 0;
            double stoneSize = 8;
            double fixedAmount = 47;
            double toFoundationSide = fixedAmount + (skystone * stoneSize);
            double fromFoundationSide = toFoundationSide + (3 * stoneSize) + FudgeFactor;
            double toFoundation = 10;


            // back up after getting stone
            encoderDriveHeading.StartAction(0.9, -6-extra, 0, 5, true);
            //sleep(1000);
            // drive to foundation side
        /*
            Strafing.Strafe(1, toFoundationSide + 2, 5, true);
            rotateToHeading.DoIt(0);
         */

            rotateToHeading.DoIt(headingduh);
            robot.platformy.setPosition(0.57);

            encoderDriveHeading.StartAction(0.9, -toFoundationSide, headingduh, 10, true);

            // drop off block

        /*
            robot.rightIntake.setPower(-1.0);
            robot.leftIntake.setPower(1.0);
            sleep(500);
            robot.rightIntake.setPower(0.0);
            robot.leftIntake.setPower(0.0);

         */

            //arm swing yay
            robot.rightIntake.setPower(1.0);
            robot.leftIntake.setPower(-1.0);

            //sleep(300);
            robot.clawy.setPosition(clawclose);
            sleep(500);
            flippyEncoderDrive.MoveToEncoderValue(0.6, 0.5, 5, false);

            sleep(500);
            robot.rightIntake.setPower(0.0);
            robot.leftIntake.setPower(0.0);
            robot.clawy.setPosition(clawopen);

            sleep(400);
            robot.clawy.setPosition(clawclose);
            sleep(400);
            flippyEncoderDrive.MoveToEncoderValue(0.3, 0.0, 5, false);
            sleep(800);
            robot.clawy.setPosition(clawopen);
            //rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.9, toFoundationSide+1.5+stoneSize, headingduh, 5, true);

            // get next block
            //Strafing.Strafe(1, 23, 5, true);


            rotateToHeading.DoIt(heading); // turning to grab block
            robot.platformy.setPosition(platformDownPos);
            robot.rightIntake.setPower(1.0);
            robot.leftIntake.setPower(-1.0);
            encoderDriveHeading.StartAction(0.3, 15,  heading, 2, true);
            sleep(300);
            //robot.rightIntake.setPower(0.0);
            //robot.leftIntake.setPower(0.0);
            encoderDriveHeading.StartAction(0.9, -15, heading, 5, true);

            //robot.rightIntake.setPower(0.0);
            //robot.leftIntake.setPower(0.0);
            //sleep(300);
            rotateToHeading.DoIt(headingduh);

            // go back to foundation
            //Strafing.Strafe(0.6, fromFoundationSide, 5, true);
            //rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.9, -fromFoundationSide, headingduh, 5, true);
            rotateToHeading.DoIt(180);
            //slowly drive up and get foundation
            encoderDriveHeading.StartAction(0.3, -toFoundation + 0, 180, 5, true);


            //encoderDriveHeading.StartAction(0.8, -5, 180, 5, true);
            robot.rightGrabber.setPosition(0.07);
            robot.leftGrabber.setPosition(0.07);
            /*
            robot.rightIntake.setPower(-1.0);
            robot.leftIntake.setPower(1.0);

            sleep(350);

            robot.rightIntake.setPower(0);
            robot.leftIntake.setPower(0);

             */
            robot.platformy.setPosition(0.57);
            sleep(350);

            robot.rightIntake.setPower(1.0);
            robot.leftIntake.setPower(-1.0);
            robot.platformy.setPosition(0.26);

            //These lines will spin the foundation
            encoderDriveHeading.StartAction(1.0, 30, 180, 5, true);
            robot.clawy.setPosition(0.45);
            rotatePrecise.RotatePrecise(-headingduh, 2, 0.6, 0.3, 2);
            encoderDriveHeading.StartAction(1.0, -5, -90, 5, true);

            // swing arm yay
            flippyEncoderDrive.MoveToEncoderValue(0.6, 0.5, 5, false);
            sleep(300);
            robot.rightIntake.setPower(0.0);
            robot.leftIntake.setPower(0.0);
            robot.clawy.setPosition(clawopen); //open

            robot.rightGrabber.setPosition(0.75);
            robot.leftGrabber.setPosition(0.75);

            sleep(350);

            robot.clawy.setPosition(clawclose);
            sleep(200);
            flippyEncoderDrive.MoveToEncoderValue(0.3, 0.01, 5, false);
            sleep(200);
            //encoderDrive.StartAction(1.0, 7, 7, 5, true);

            rotateToHeading.DoIt(30);

            //Strafing.Strafe(1.0, 12, 5, true);

            //Add code here to park on line
            encoderDriveHeading.StartAction(1.0, 37, 30, 5, true);


        }
    }


