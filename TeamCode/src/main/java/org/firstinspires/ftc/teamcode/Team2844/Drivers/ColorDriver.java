package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ColorDriver {
    private PushbotRobotHardware robot_;

    public ColorDriver(PushbotRobotHardware robot) {
        robot_ = robot;

    }

    public boolean isSeen() {
        return  !new Double(robot_.distancedriver.getDistance(DistanceUnit.MM)).isNaN();
    }

    public boolean isYellow() {

        //insert color hue test
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (robot_.colordriver.red() * SCALE_FACTOR),
                (int) (robot_.colordriver.green() * SCALE_FACTOR),
                (int) (robot_.colordriver.blue() * SCALE_FACTOR),
                hsvValues);

        return (hsvValues[0] >= 40 && hsvValues[0] <= 70);
    }
}