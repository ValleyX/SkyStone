package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "ColorDistanceSensor", group = "Sensor")
public class ColorDriver extends LinearOpMode{

    private PushbotRobotHardware robot_;

    private ColorSensor senseColor;
    @Override
    public void runOpMode() throws InterruptedException {

        //Maps Sensor to the REV Hub
        senseColor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

        //insert color hue test
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

        while (opModeIsActive()) {
            Color.RGBToHSV((int) (senseColor.red() * SCALE_FACTOR),
                    (int) (senseColor.green() * SCALE_FACTOR),
                    (int) (senseColor.blue() * SCALE_FACTOR),
                    hsvValues);

            senseColor.red();
            senseColor.blue();
            senseColor.green();

            telemetry.addData("isYellow", hsvValues[0] >= 40 && hsvValues[0] <= 70);

            telemetry.addData("Red", senseColor.red());
            telemetry.addData("Blue", senseColor.blue());
            telemetry.addData("Green", senseColor.green());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.update();
        }
    }
}