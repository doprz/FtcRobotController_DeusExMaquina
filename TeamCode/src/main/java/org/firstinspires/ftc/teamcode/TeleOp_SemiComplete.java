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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.enums.ClampServoState;


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

@TeleOp(name="TeleOp SemiComplete", group="Linear Opmode")
//@Disabled
public class TeleOp_SemiComplete extends LinearOpMode {

    // State of the wobble goal claw servo
    //private enum ClampServoState {
    //    OPEN,
    //    CLOSED
    //}

    //========================================
    // DECLARE OPMODE MEMBERS
    //========================================

    // Misc
    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain Motors
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private boolean drivetrainRegularSpeed = true;
    private static final double DRIVETRAIN_REDUCED_SPEED_COEFFICIENT = 2.0; // Should be a value n > 1

    /* Drivetrain Constants */
    private static final double STRAFING_SENSIBILITY = 1.5;

    // The higher the number the more sensitive and the smaller the number the less sensitive the drivetrain controls are
    private static final double GAMEPAD1_DRIVETRAIN_SENSITIVITY = 1.0;


    // Shooter Motors
    //private DcMotor shooterMotor1 = null;
    //private DcMotor shooterMotor2 = null;

    // Wobble Goal Clamp Servo
    private Servo clampServo = null;

    // State of the wobble goal clamp servo
    private ClampServoState clampServo_state = ClampServoState.CLOSED;
    /* Wobble Goal Clamp Default Values and Constants */
    private static final double CLAMPSERVO_OPENPOSITION = 0.7;

    // Wobble Goal Arm Motor
    private DcMotor wobbleGoalArmMotor = null;
    private static final double WOBBLE_GOAL_ARM_MOTOR_POWER = 0.2;

    private boolean wobbleGoalArmMotorRegularPower = true;
    private static final double WOBBLE_GOAL_ARM_MOTOR_REDUCED_POWER_COEFFICIENT = 2.0; // Should be a value n > 1

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

        /*
        * Motors
        * */

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

        /*
         * Shooter Motors
         * */

        //shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        //shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");

        /*
         * Wobble Goal Clamp Servo
         * */

        // Hardware map the servo object to the actual servo
        clampServo = hardwareMap.servo.get("clampServo");
        // Reset the servo's position to 0 degrees
        clampServo.setPosition(0.0);

        /*
         * Wobble Goal Motor
         * */

        wobbleGoalArmMotor = hardwareMap.get(DcMotor.class, "wobbleGoalArmMotor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //========================================
            // MECANUM DRIVETRAIN
            //========================================

            double y = -gamepad1.left_stick_y * GAMEPAD1_DRIVETRAIN_SENSITIVITY;                            // This is reversed
            double x = (gamepad1.left_stick_x * STRAFING_SENSIBILITY) * GAMEPAD1_DRIVETRAIN_SENSITIVITY;    // Counteract strafing imperfections
            double rx = gamepad1.right_stick_x * GAMEPAD1_DRIVETRAIN_SENSITIVITY;                           // Strafing

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Scale values in between -1.0 and 1.0 to prevent the clipping of values in order to maintain the driving ratio
            if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1 ) {

                // Find the largest value
                double max = 0;
                max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                max = Math.max(Math.abs(frontRightPower), max);
                max = Math.max(Math.abs(backRightPower), max);

                // Divide everything by max (it's positive so we don't need to worry about signs)
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            /* Toggle the robot's drivetrain between a regular and reduced speed movement state */
            if (gamepad1.left_bumper) {
                // Regular speed
                drivetrainRegularSpeed = true;
                telemetry.addData("Drivetrain Mode", "Regular");
                telemetry.update();
            } else if (gamepad1.right_bumper) {
                // Slower speed
                drivetrainRegularSpeed = false;
                telemetry.addData("Drivetrain Mode", "Reduced Speed");
                telemetry.update();
            }

            if (drivetrainRegularSpeed) {
                // Update Drivetrain motors at a regular speed
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            } else {
                // Update Drivetrain motors at a reduced speed
                frontLeftMotor.setPower(frontLeftPower / DRIVETRAIN_REDUCED_SPEED_COEFFICIENT);
                backLeftMotor.setPower(backLeftPower / DRIVETRAIN_REDUCED_SPEED_COEFFICIENT);
                frontRightMotor.setPower(frontRightPower / DRIVETRAIN_REDUCED_SPEED_COEFFICIENT);
                backRightMotor.setPower(backRightPower / DRIVETRAIN_REDUCED_SPEED_COEFFICIENT);
            }

            //========================================
            // Wobble Goal Arm
            //========================================

            /* To do list:
            - Redesign and use an "elevator instead"
             */

            /* Clamp */
            if (gamepad2.right_trigger > 0) {
                clampServo.setPosition(gamepad2.right_trigger);
            } else {
                // Toggle the wobble goal claw servo
                if (gamepad2.right_bumper && (clampServo_state == ClampServoState.CLOSED)) {
                    // Open
                    clampServo.setPosition(CLAMPSERVO_OPENPOSITION);
                    clampServo_state = ClampServoState.OPEN;
                } else if (gamepad2.left_bumper && (clampServo_state == ClampServoState.OPEN)) {
                    // Close
                    clampServo.setPosition(0.0);
                    clampServo_state = ClampServoState.CLOSED;
                }
            }

            /* Arm Motor */

            /* Toggle the robot's arm motor between a regular and reduced power state */
            if (gamepad2.a) {
                // Regular speed
                wobbleGoalArmMotorRegularPower = true;
                telemetry.addData("Arm Power", "Regular");
                telemetry.update();
            } else if (gamepad2.y) {
                // Slower speed
                wobbleGoalArmMotorRegularPower = false;
                telemetry.addData("Arm Power", "Reduced Power");
                telemetry.update();
            }

            // Toggle in between regular and reduced power for the wobble goal arm motor
            if (wobbleGoalArmMotorRegularPower) {
                // Regular Power
                if (gamepad2.dpad_up) {
                    wobbleGoalArmMotor.setPower(WOBBLE_GOAL_ARM_MOTOR_POWER);
                } else if (gamepad2.dpad_down) {
                    wobbleGoalArmMotor.setPower(-WOBBLE_GOAL_ARM_MOTOR_POWER);
                } else {
                    // Reset and stop the motor
                    wobbleGoalArmMotor.setPower(0);
                }
            } else {
                // Reduced Power
                // Wobble goal arm motor reduced speed/power
                double wobbleGoalArmMotorReducedPower = WOBBLE_GOAL_ARM_MOTOR_POWER / WOBBLE_GOAL_ARM_MOTOR_REDUCED_POWER_COEFFICIENT;

                if (gamepad2.dpad_up) {
                    wobbleGoalArmMotor.setPower(wobbleGoalArmMotorReducedPower);
                } else if (gamepad2.dpad_down) {
                    wobbleGoalArmMotor.setPower(-wobbleGoalArmMotorReducedPower);
                } else {
                    // Reset and stop the motor
                    wobbleGoalArmMotor.setPower(0);
                }
            }


            //========================================
            // SHOOTER
            //========================================
            /* To do list:
            - Intake system
            - Keep track of how many rings at all times
            - There's a servo/something that pushes rings
            - Shooter/Motors need to turn on when the game starts
             */

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), back left (%.2f), front right (%.2f), back right (%.2f)", frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}
