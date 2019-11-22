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

package org.firstinspires.ftc.teamcode.Team12841;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDriveStrafe;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

/**
 * This file is designed to test out the EncoderDrive class
 */

@Autonomous(name="Autonomous Base Blue", group="Test")
//@Disabled
public class AutonomousBaseBlue extends LinearOpMode {

    /* Declare OpMode members. */
    public final double Latch_UP_POSITION = -0.25;
    public final double Latch_DOWN_POSITION = 0.70;
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
        System.out.println("ValleyX: initial strafe left");
        encoderDriveStrafe.StartAction( .3, -13, 13, 13, -13,
                4, true);

        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Move forward 12 inches");
        //going forwards 12 inches
        encoderDriveStrafe.StartAction(0.5, 31.5, 31.5, 31.5, 31.5,
                4, true);

        sleep(50);
        //going backwards 12 inches
        System.out.println("ValleyX: Latch Down");
        robot.BackRightHook.setPosition(Latch_DOWN_POSITION);
        robot.BackLeftHook.setPosition(Latch_UP_POSITION);
        idle();

        sleep(50);
        //going backwards 12 inches
        System.out.println("ValleyX: Move backwards 12 inches");
        encoderDriveStrafe.StartAction(0.5, -31.5, -31.5, -31.5, -31.5,
                4, true);

        sleep(50);
        //going backwards 12 inches
        System.out.println("ValleyX: Latch Up");
        robot.BackRightHook.setPosition(Latch_UP_POSITION);
        robot.BackLeftHook.setPosition(Latch_DOWN_POSITION);
        idle();

        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Move forward 12 inches");
        //going forwards 12 inches
        encoderDriveStrafe.StartAction(0.1, .2, .2, .2, .2,
                4, true);

        sleep(50);
        //strafe right about 20 inches
        System.out.println("ValleyX: Strafe Right 20 inches");
        encoderDriveStrafe.StartAction(0.3, 37, -37, -37, 37,
                4, true);

        sleep(50);
        //going forward 18 inches
        System.out.println("ValleyX: Move forward 18 inches");
        encoderDriveStrafe.StartAction(.5, 18, 18, 18, 18,
                4, true);

        sleep(50);
        //strafe left foundation push
        System.out.println("ValleyX: Strafe Final Building Foundation Push left");
        encoderDriveStrafe.StartAction( .3, -29, 29, 29, -29,
                4, true);

        sleep(50);
        //go backwards to wall
        System.out.println("ValleyX: Move Back 18 inches");
        encoderDriveStrafe.StartAction(.5, -16.8, -16.8, -16.8, -16.8,
                3, true);
        sleep(50);
        //strafe park on line
        System.out.println("ValleyX: Strafe right to line");
        encoderDriveStrafe.StartAction(.5, 38,-38,-38,38,
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




