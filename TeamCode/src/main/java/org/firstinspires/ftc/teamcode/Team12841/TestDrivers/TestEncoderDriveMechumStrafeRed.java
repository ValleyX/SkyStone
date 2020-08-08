/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDriveStrafe;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

/**
 * This file is designed to test out the EncoderDrive class
 */

@Autonomous(name="Test: Best Strafe Encoder Drive Red", group="Test")
@Disabled
public class TestEncoderDriveMechumStrafeRed extends LinearOpMode {

    /* Declare OpMode members. */


    @Override
    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();

        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDriveStrafe encoderDriveStrafe = new EncoderDriveStrafe(robot);

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(50);
        //strafe left foundation push
        System.out.println("ValleyX: initial strafe right");
        encoderDriveStrafe.StartAction( .4, 7.86, -7.86, -7.86, 7.86,
                4, true);

        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Move forward 12 inches");
        //going forwards 12 inches
        encoderDriveStrafe.StartAction(0.7, 31.5, 31.5, 31.5, 31.5,
                4, true);

        sleep(50);
        //going backwards 12 inches
        System.out.println("ValleyX: Move backwards 12 inches");
        encoderDriveStrafe.StartAction(0.7, -31.5, -31.5, -31.5, -31.5,
                4, true);

        sleep(50);
        //strafe left about 20 inches
        System.out.println("ValleyX: Strafe Left 20 inches");
        encoderDriveStrafe.StartAction(0.4, -35.66, 35.66, 35.66, -35.66,
                4, true);

        sleep(50);
        //going forward 18 inches
        System.out.println("ValleyX: Move forward 18 inches");
        encoderDriveStrafe.StartAction(.7, 18, 18, 18, 18,
                4, true);

        sleep(50);
        //strafe left foundation push
        System.out.println("ValleyX: Strafe Final Building Foundation Push right");
        encoderDriveStrafe.StartAction( .4, 28.5, -28.5, -28.5, 28.5,
                4, true);

        sleep(50);
        //go backwards to wall
        System.out.println("ValleyX: Move Back 18 inches");
        encoderDriveStrafe.StartAction(.7, -16.8, -16.8, -16.8, -16.8,
                3, true);
        sleep(50);
        //strafe park on line
        System.out.println("ValleyX: Strafe left to line");
        encoderDriveStrafe.StartAction(.5, -36,36,36,-36,
                4,true);

       /* sleep(2000);
        //testing no wait functions going forward 12 inches
        runtime.reset();
        System.out.println("ValleyX: Move forward 12 inches");
        encoderDrive.StartAction(0.6, 12, 12, 6.0, false);

        //spin here until encoder is complete
        while (opModeIsActive() && !encoderDrive.IsActionDone() && runtime.seconds() < 5.0)
        {
           idle();
        }

        //Spin above is completed
        encoderDrive.StopAction(); //stop all motors started by StartAction*/
    }

}




