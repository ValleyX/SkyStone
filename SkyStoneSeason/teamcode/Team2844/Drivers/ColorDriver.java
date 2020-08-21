package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ColorDriver {
    private TestRobotHardware robot_;

    public ColorDriver(TestRobotHardware robot) {
        robot_ = robot;

    }

    public boolean isSeen() {
        return  !new Double(robot_.distanceDriver.getDistance(DistanceUnit.MM)).isNaN();
    }

    public boolean isYellow() {

        //insert color hue test
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (robot_.colorDriver.red() * SCALE_FACTOR),
                (int) (robot_.colorDriver.green() * SCALE_FACTOR),
                (int) (robot_.colorDriver.blue() * SCALE_FACTOR),
                hsvValues);

        return (hsvValues[0] >= 30 && hsvValues[0] <= 70);
    }
}