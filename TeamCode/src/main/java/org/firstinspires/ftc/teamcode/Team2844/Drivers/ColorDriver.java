package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ColorDriver {
    private PushbotRobotHardware robot_;
    //private ColorSensor senseColor;

    public ColorDriver(PushbotRobotHardware robot) {
        robot_ = robot;

    }


    public boolean isYellow() {

        //Maps Sensor to the REV Hub
        //senseColor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

        //insert color hue test
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

       // int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
       // final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //waitForStart();

        //while (opModeIsActive()) {
            Color.RGBToHSV((int) (robot_.colordriver.red() * SCALE_FACTOR),
                    (int) (robot_.colordriver.green() * SCALE_FACTOR),
                    (int) (robot_.colordriver.blue() * SCALE_FACTOR),
                    hsvValues);
            /*
            robot_.OpMode_.telemetry.addData("isYellow", hsvValues[0] >= 40 && hsvValues[0] <= 70);

            robot_.OpMode_.telemetry.addData("Red", robot_.colordriver.red());
            robot_.OpMode_.telemetry.addData("Blue", robot_.colordriver.blue());
            robot_.OpMode_.telemetry.addData("Green", robot_.colordriver.green());
            robot_.OpMode_.telemetry.addData("Hue", hsvValues[0]);

            robot_.OpMode_.telemetry.update();

             */
            return (hsvValues[0] >= 40 && hsvValues[0] <= 70);
    }
}