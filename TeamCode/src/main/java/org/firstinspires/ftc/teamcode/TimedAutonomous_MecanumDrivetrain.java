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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Timed Autonomous Mecanum Drivetrain OpMode", group="Linear Opmode")
//@Disabled
public class TimedAutonomous_MecanumDrivetrain extends LinearOpMode {

    //========================================
    // DECLARE OPMODE MEMBERS
    //========================================

    // Misc
    private ElapsedTime runtime = new ElapsedTime();
    private static boolean autonomousRunning = true;

    // Motors
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    // Servos

    // Constants
    private static final double APPROACH_SPEED = 1;
    private static final long DRIVE_TO_LINE_TIME = 2000;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //========================================
        // HARDWARE MAPPING
        //========================================

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //========================================
        // AUTONOMOUS STARTS
        //========================================

        moveRobotByTime(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, runtime, DRIVE_TO_LINE_TIME, APPROACH_SPEED);

        // Stop the robot after it parks on the line
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive() && runtime.milliseconds() < DRIVE_TO_LINE_TIME) {
        //
        //    //========================================
        //    // Autonomous Mode
        //    //========================================
        //
        //    // Park on the line through timing
        //    frontLeftMotor.setPower(APPROACH_SPEED);
        //    backLeftMotor.setPower(APPROACH_SPEED);
        //    frontRightMotor.setPower(APPROACH_SPEED);
        //    backRightMotor.setPower(APPROACH_SPEED);
        //
        //    // Show the elapsed game time and wheel power.
        //    telemetry.addData("Status", "Run Time: " + runtime.toString());
        //    //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        //    telemetry.update();
        //}

        // Stop all motors at the end of the autonomous period
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

    }

    /**
     * Handles the autonomous movement of the robot through timing
     *
     * @param frontLeftMotor    Pass by reference the frontLeftMotor object
     * @param backLeftMotor     Pass by reference the backLeftMotor object
     * @param frontRightMotor   Pass by reference the frontRightMotor object
     * @param backRightMotor    Pass by reference the backRightMotor object
     *
     * @param runTime           Pass by reference the ElapsedTime object from the match
     * @param time              The time (in milliseconds) that the robot should travel for
     *
     * @param speed             The speed for all of the motors
     * */
    private void moveRobotByTime(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, ElapsedTime runTime, long time, double speed) {
        double startRunTime = runTime.milliseconds();

        while (opModeIsActive() && ((startRunTime + time) < runTime.milliseconds())) {

            frontLeftMotor.setPower(speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(speed);
            backRightMotor.setPower(speed);

        }
    }

}
