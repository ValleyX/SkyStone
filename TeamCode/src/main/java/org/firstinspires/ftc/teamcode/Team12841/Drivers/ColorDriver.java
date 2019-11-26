package org.firstinspires.ftc.teamcode.Team12841.Drivers;

import android.graphics.Color;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

public class ColorDriver {

    private RobotHardwarePushbot robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private int hsvValues;

     public ColorDriver(RobotHardwarePushbot robot) {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
    }

    public boolean IsYellow() {
         return false;
    }
}
