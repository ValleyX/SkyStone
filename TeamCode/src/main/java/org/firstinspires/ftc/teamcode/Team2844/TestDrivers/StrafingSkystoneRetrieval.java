package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.DriveTo;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.StrafingEncoderDrive;
import org.firstinspires.ftc.teamcode.dogecvDetectors.BlackRectDetector;
import org.firstinspires.ftc.teamcode.dogecvDetectors.SkyStone2Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//import org.firstinspires.ftc.teamcode.Team2844.Drivers.ColorDriver;


@Autonomous(name = "Test: StrafingSkystoneRetrieval", group ="Test")
//@Disabled
public class StrafingSkystoneRetrieval extends LinearOpMode {
    //ColorDriver colorDriver;
    EncoderDrive encoderDrive;
    RobotHardware robot;
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
        //colorDriver = new ColorDriver(robot);

        //code for test bot only
        RotatePrecise rotatePrecise = new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);
        DriveTo driveTo = new DriveTo(robot, encoderDrive);

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

        sleep(5000);

        waitForStart();
            //encoderDrive.StartAction(.5, -28, -28, 5, true);

            //test code
            //rotateToHeading.DoIt(0);
            //driveTo.StartAction(0.5, 1.5, 5, true);
            //test code

/*
            sleep(1000);
            skystone = 0;
            while (colorDriver.isSeen() && colorDriver.isYellow() && skystone < 2)
            {
                Strafing.Strafe(.5, -10, 5, true);

                //test code
                rotateToHeading.DoIt(0);
                driveTo.StartAction(0.5, 1.5, 5, true);
                //test code

                telemetry.addData("Skystonenotfound", skystone);
                telemetry.update();
                skystone += 1;
                sleep(1000);
            }
            if (colorDriver.isSeen() && !colorDriver.isYellow()){
                telemetry.addData("Skystoneisfound", skystone);
                telemetry.update();
            } else{
                telemetry.addData("NOTHING", skystone);
                telemetry.update();
            }
            sleep(100);
            break;
        }

 */

            while(opModeIsActive()) {

                robot.swingy.setPosition(0.87);

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
                        Strafing.Strafe(0.6, -9, 5, true);
                        rotateToHeading.DoIt(0);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDrive.StartAction(0.6, 48, 48, 5, true);
                        sleep(200);
                        robot.rightIntake.setPower(0.0);
                        robot.leftIntake.setPower(0.0);
                        break;
                    }
                    if (leftXLine < stoneXValue && stoneXValue < rightXLine) {
                        telemetry.addData("stone found middle", stoneXValue);
                        System.out.println("ValleyX stone found middle " + stoneXValue);
                        skystone = 1;
                        robot.platformy.setPosition(0.26);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDrive.StartAction(0.6, 48, 48, 5, true);
                        sleep(200);
                        robot.rightIntake.setPower(0.0);
                        robot.leftIntake.setPower(0.0);
                        break;
                    }
                    if (stoneXValue > rightXLine) {
                        telemetry.addData("stone found right", stoneXValue);
                        System.out.println("ValleyX stone found right " + stoneXValue);
                        skystone = 0;
                        encoderDrive.StartAction(1.0, 2, 2, 5, true);
                        Strafing.Strafe(0.6, 9, 5, true);
                        rotateToHeading.DoIt(0);
                        robot.platformy.setPosition(0.26);
                        robot.rightIntake.setPower(1.0);
                        robot.leftIntake.setPower(-1.0);
                        encoderDrive.StartAction(0.6, 46, 46, 5, true);
                        sleep(200);
                        robot.rightIntake.setPower(0.0);
                        robot.leftIntake.setPower(0.0);
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

            double FudgeFactor = 7;
            double stoneSize = 8;
            double fixedAmount = 45;
            double toFoundationSide = fixedAmount + (skystone * stoneSize);
            double fromFoundationSide = toFoundationSide + (3 * stoneSize) + FudgeFactor;
            double toFoundation = 10;


            // back up after getting stone
            encoderDrive.StartAction(1.0, -24, -24, 5, true);

            // drive to foundation side
            Strafing.Strafe(0.6, toFoundationSide, 5, true);
            rotateToHeading.DoIt(0);

            // drop off block
            robot.rightIntake.setPower(-1.0);
            robot.leftIntake.setPower(1.0);
            sleep(1000);
            robot.rightIntake.setPower(0.0);
            robot.leftIntake.setPower(0.0);

            /*
            Strafing.Strafe(1.0, -fromFoundationSide, 5, true);
            rotateToHeading.DoIt(0);
             */

            rotateToHeading.DoIt(-90);
            encoderDrive.StartAction(1.0, fromFoundationSide - 32, fromFoundationSide - 32, 5, true);

            // get next block
            Strafing.Strafe(0.6, 25, 5, true);
            robot.rightIntake.setPower(1.0);
            robot.leftIntake.setPower(-1.0);
            encoderDrive.StartAction(0.6, 7, 7, 5, true);
            sleep(300);
            robot.rightIntake.setPower(0.0);
            robot.leftIntake.setPower(0.0);
            Strafing.Strafe(0.6, -20, 5, true);

            // go back to foundation
            //Strafing.Strafe(0.6, fromFoundationSide, 5, true);
            rotateToHeading.DoIt(-90);
            encoderDrive.StartAction(1.0, -fromFoundationSide - 20, -fromFoundationSide - 20, 5, true);
            rotateToHeading.DoIt(180);
            encoderDrive.StartAction(0.6, -toFoundation, -toFoundation, 5, true);

            // drag foundation back
        }
    }


