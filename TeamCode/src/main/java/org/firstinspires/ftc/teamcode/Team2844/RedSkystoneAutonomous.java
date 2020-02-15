package org.firstinspires.ftc.teamcode.Team2844;

import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous(name = "RedSkystoneAutonomous", group ="Test")
//@Disabled
public class RedSkystoneAutonomous extends LinearOpMode {
    EncoderDrive encoderDrive;
    RobotHardware robot;
    FlippyDriver flippy;
    EncoderDriveHeading encoderDriveHeading;
    SkyStone2Detector skystoneDetector;
    StoneDetector stoneDetector;
    BlackRectDetector blackRectDetector;
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        StrafingEncoderDrive Strafing = new StrafingEncoderDrive(robot);
        FlippyDriver flippy = new FlippyDriver(robot);

        EncoderDriveHeading encoderDriveHeading = new EncoderDriveHeading(robot);
        flippyEncoderDrive flippyEncoderDrive = new flippyEncoderDrive(robot);

        //code for real bot only
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);

        skystoneDetector = new SkyStone2Detector();
        stoneDetector = new StoneDetector();
        blackRectDetector = new BlackRectDetector();

        int leftXLine = 120; // purple
//        int leftXLine = 115; // purple
        int rightXLine = 187; // green

        skystoneDetector.SetRequestedYLine(205);
        skystoneDetector.SetRequestedXRightLine(290);
        //skystoneDetector.SetRequestedXLeftLine(50);
        skystoneDetector.SetRequestedXLeftLine(80);
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

        double clawopen = 0.1; //0.0
        double clawclose = 0.45; //0.37 //0.55

        robot.rightGrabber.setPosition(0.4);
        robot.leftGrabber.setPosition(0.4);
        final double platformDownPos = 0.27;
        final double platformyFlat = 0.57;
        robot.platformy.setPosition(platformDownPos);
        robot.clawy.setPosition(clawopen);
        flippyEncoderDrive.MoveToEncoderValue(0.2, 0.01, 5, true);


        final int headingduh = -90;
        final int heading = -45;


        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ElapsedTime runtime;
        runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        //double extra = 0;



        while (opModeIsActive())
        {
            /* old
                if (skystoneDetector.isDetected()) {
                    //webcam.setPipeline(null);
                    phoneCam.setPipeline(null);
                    telemetry.addData("Skystone found X Y", "%d %d",
                            skystoneDetector.foundRectangle().x, skystoneDetector.foundRectangle().y);
                    int stoneXValue = skystoneDetector.foundRectangle().x;
                    if (leftXLine > stoneXValue) {
                        telemetry.addData("stone found left", stoneXValue);
                        System.out.println("ValleyX stone found left " + stoneXValue);
                        skystone = 2;
                        //rotateToHeading.DoIt(0);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        extra = 0;
                        encoderDriveHeading.StartAction(0.9, 30, -15, 5, true);
                        encoderDriveHeading.StartAction(0.3, 14, 50, 1, true);
                        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        sleep(400);
                        robot.platformy.setPosition(platformyFlat);
                        encoderDriveHeading.StartAction(0.9, -14.5, 0, 5, true);

                        //robot.rightIntake.setPower(0.0);
                        //robot.leftIntake.setPower(0.0);
                        break;
                    }
                    if (leftXLine < stoneXValue && stoneXValue < rightXLine) {
                        telemetry.addData("stone found middle", stoneXValue);
                        System.out.println("ValleyX stone found middle " + stoneXValue);
                        skystone = 1;


                        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);

                        encoderDriveHeading.StartAction(0.9, 27, 0, 5, true);
                        encoderDriveHeading.StartAction(0.3, 13, 0, 1, true);
                        sleep(400);
                        robot.platformy.setPosition(platformyFlat);
                        encoderDriveHeading.StartAction(0.9, -12.5, 0, 5, true);

                        break;
                    }
                    if (stoneXValue > rightXLine) {
                        telemetry.addData("stone found right", stoneXValue);
                        System.out.println("ValleyX stone found right " + stoneXValue);
                        skystone = 0;

                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDriveHeading.StartAction(0.9, 30, 15, 5, true);
                        encoderDriveHeading.StartAction(0.3, 14, -50, 1, true);
                        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        sleep(400);
                        robot.platformy.setPosition(platformyFlat);
                        encoderDriveHeading.StartAction(0.9, -14.5, 0, 5, true);

                        break;
                    }
                } else {
                    telemetry.addLine("Sky stone not found");
                }*/
                //new
                if (skystoneDetector.isDetected()) {
                    //webcam.setPipeline(null);
                    phoneCam.setPipeline(null);
                    telemetry.addData("Skystone found X Y", "%d %d",
                            skystoneDetector.foundRectangle().x, skystoneDetector.foundRectangle().y);
                    int stoneXValue = skystoneDetector.foundRectangle().x;
                    if (leftXLine > stoneXValue) {
                        telemetry.addData("stone found left", stoneXValue);
                        System.out.println("ValleyX stone found left " + stoneXValue);
                        skystone = 2;
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDriveHeading.StartAction(0.5, 30, -15, 5, true);
                        encoderDriveHeading.StartAction(0.3, 5, 50, 1, true);
                        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        sleep(400);
                        robot.platformy.setPosition(platformyFlat);
                        encoderDriveHeading.StartAction(0.9, -5.5, 0, 5, true);

                        break;
                    }
                    if (leftXLine < stoneXValue && stoneXValue < rightXLine) {
                        telemetry.addData("stone found middle", stoneXValue);
                        System.out.println("ValleyX Auto: stone found middle " + stoneXValue + " " + runtime.milliseconds());
                        skystone = 1;

                        //robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        //robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        //robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        //robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);

                        encoderDriveHeading.StartAction(0.5, 27, 0, 5, true);
                        System.out.println("ValleyX Auto: get stone " + runtime.milliseconds());

                        encoderDriveHeading.StartAction(0.3, 15, 0, 2, true);

                        System.out.println("ValleyX Auto: suck " + runtime.milliseconds());
                        sleep(400);
                        robot.platformy.setPosition(platformyFlat);
                        System.out.println("ValleyX Auto: go back " + runtime.milliseconds());

                        encoderDriveHeading.StartAction(0.8, -13, 0, 5, true);

                        break;
                    }
                    if (stoneXValue > rightXLine) {
                        telemetry.addData("stone found right", stoneXValue);
                        System.out.println("ValleyX stone found right " + stoneXValue);
                        skystone = 0;

                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDriveHeading.StartAction(0.5, 30, 15, 5, true);
                        encoderDriveHeading.StartAction(0.3, 5, -50, 1, true);
                        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        sleep(400);
                        robot.platformy.setPosition(platformyFlat);
                        encoderDriveHeading.StartAction(0.9, -5.5, 0, 5, true);

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


            double FudgeFactor = 1;
            double stoneSize = 8;
            double fixedAmount = 68;
            double toFoundationSide = fixedAmount + (skystone * stoneSize);
            double fromFoundationSide = toFoundationSide + (3 * stoneSize) + FudgeFactor;
            double toFoundation = 13;
        //double toGlass = (3 * stoneSize) - (skystone * stoneSize) - 8;
            double toGlass =  (2-skystone) * stoneSize;

            System.out.println("ValleyX Auto: Got first stone and backed up " + runtime.milliseconds());

        // have driven up and gotten the first skystone and backed up (put platform down)

            //drop arm on top of the intaken block
            flippyEncoderDrive.MoveToEncoderValue(1.0, 0.00, 5, false); //TODO consider making false to save time

            //rotateToHeading.DoIt(headingduh);

            //faster spin to drive backwards to foundation side
            rotateToHeading.DoItSpecify(headingduh, 4, 0.7, 0.3, 4);

            // claw closing while drive backwards
            robot.clawy.setPosition(clawclose);

            System.out.println("ValleyX Auto: Going to foundation " + runtime.milliseconds());

        // driving backwards to foundation side
            encoderDriveHeading.StartAction(.8, -toFoundationSide, headingduh, 10, true);

            robot.rightIntake.setPower(0);
            robot.leftIntake.setPower(0);

            System.out.println("ValleyX Auto: Spinning to foundation " + runtime.milliseconds());

        // spinning while moving the arm up to save time
            flippyEncoderDrive.MoveToEncoderValue(1.0, 0.47, 5, false); //spin @ same time
            //rotateToHeading.DoItSpecify(180, 3, 0.4, 0.3, 4);
            rotateToHeading.DoIt(180);
            sleep(200);

            System.out.println("ValleyX Auto: Drop first block " + runtime.milliseconds());

        // open claw to drop block and close again to swing back over (will drop block from a higher height)
            robot.clawy.setPosition(clawopen);
            sleep(600);  //maybe able to make smaller
            robot.clawy.setPosition(clawclose);

            System.out.println("ValleyX Auto: Spinning to go back " + runtime.milliseconds());

        //faster spin to drive forwards to skystone side
            rotateToHeading.DoItSpecify(headingduh, 4, 0.7, 0.3, 4);

        System.out.println("ValleyX Auto: Go back " + runtime.milliseconds());

        // moving arm down while driving back to skystone side
            flippyEncoderDrive.MoveToEncoderValue(1.0, 0.0, 5, false);
            encoderDriveHeading.StartAction(.8, fromFoundationSide, headingduh, 5, true);
            robot.clawy.setPosition(clawopen); // to intake next block

            System.out.println("ValleyX Auto: Before moving to glass " + runtime.milliseconds());
            //move to glass
            robot.platformy.setPosition(platformDownPos);
            encoderDriveHeading.StartAction(0.9, toGlass, headingduh, 1, true);

            System.out.println("ValleyX Auto: Backing up to stone " + runtime.milliseconds());

             //do this for middle and right (on skystone side)
            encoderDriveHeading.StartAction(0.8, ((2-skystone) * -stoneSize) + 6, headingduh, 1.5, true);

            System.out.println("ValleyX Auto: rotate to stone " + runtime.milliseconds());

            //turning to face blocks
            rotateToHeading.DoIt(0);
            //rotateToHeading.DoItSpecify(0, 2, 0.5, 0.3, 5);

            System.out.println("ValleyX Auto: get stone " + runtime.milliseconds());

            // getting second skystone block
            robot.rightIntake.setPower(1.0);
            robot.leftIntake.setPower(-1.0);
            encoderDriveHeading.StartAction(0.3, 15,  0, 2, true);
            sleep(400);
            robot.platformy.setPosition(platformyFlat);

           System.out.println("ValleyX Auto: Got second stone " + runtime.milliseconds());

        // backing up
            encoderDriveHeading.StartAction(0.9, -13, 0, 5, true);

            robot.rightIntake.setPower(0);
            robot.leftIntake.setPower(0);

            System.out.println("ValleyX Auto: Rotating back to foundation " + runtime.milliseconds());

        // rotating to drive backwards to foundation side
            rotateToHeading.DoItSpecify(headingduh, 4, 0.6, 0.3, 5);
            robot.clawy.setPosition(clawclose);

            System.out.println("ValleyX Auto: driving to foundation " + runtime.milliseconds());

        // driving to foundation side
            encoderDriveHeading.StartAction(0.8, -toFoundationSide - 17, headingduh, 5, true);

           System.out.println("ValleyX Auto: Rotate to foundation " + runtime.milliseconds());

        // rotating to get foundation while moving arm
            flippyEncoderDrive.MoveToEncoderValue(1.0, 0.4, 5, false);


            //rotateToHeading.DoIt(180);
            rotateToHeading.DoItSpecify(180, 4, 0.5, 0.3, 5);

            System.out.println("ValleyX Auto: Move to foundation " + runtime.milliseconds());

            // drive to foundation and latch onto it (dropping second block)
            encoderDriveHeading.StartAction(0.2, -toFoundation, 180, 1.5, true);
            robot.rightGrabber.setPosition(0.07);
            robot.leftGrabber.setPosition(0.07);
            robot.clawy.setPosition(clawopen);
            sleep(300);
            robot.clawy.setPosition(clawclose);

            System.out.println("ValleyX Auto: spin foundation " + runtime.milliseconds());

        // spin the foundation
            encoderDriveHeading.StartAction(1.0, 30, 180, 2, true);
            flippyEncoderDrive.MoveToEncoderValue(1.0, 0.0, 5, false);
            rotateToHeading.DoItSpecify(headingduh, 20, 0.6, 0.3, 2);

            // let go of foundation
            robot.rightGrabber.setPosition(0.75);
            robot.leftGrabber.setPosition(0.75);

            // drive to skybridge line
            encoderDriveHeading.StartAction(1.0, 26, headingduh+15, 5, true);
            System.out.println("ValleyX Auto: complete " + runtime.milliseconds());
        }
    }


