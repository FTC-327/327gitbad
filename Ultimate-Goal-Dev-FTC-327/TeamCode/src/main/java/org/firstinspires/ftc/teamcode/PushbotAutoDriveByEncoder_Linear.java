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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor left1Drive = null;      // 2: left front: GoBilda 5202
    private DcMotor left2Drive = null;      // 1: left back: GoBilda 5202
    private DcMotor right1Drive = null;     // 3: right front: GoBilda 5202
    private DcMotor right2Drive = null;     // 0: right back: GoBilda 5202

    private DcMotor intake = null;          // 0: Rev Hex motor Ultraplanetary
    private DcMotor intakeGear = null;      // 1: Rev Core hex motor
    private DcMotor shooter = null;         // 3: Rev Ultraplanetary

    private CRServo shooter2 = null;          // 0: servo
    private DcMotor armMotor = null;            // 2: Rev Core Hex Motor
    private Servo armServo2 = null;         //1: servo

    private NormalizedColorSensor bottomSense = null;      // Color sensor
    private NormalizedColorSensor topSense = null;        // Color sensor

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.6 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        left1Drive = hardwareMap.get(DcMotor.class, "left_front");
        left2Drive = hardwareMap.get(DcMotor.class, "left_back");
        right1Drive = hardwareMap.get(DcMotor.class, "right_front");
        right2Drive = hardwareMap.get(DcMotor.class, "right_back");

        //intake = hardwareMap.get(DcMotor.class, "intake_motor");
        //intakeGear = hardwareMap.get(DcMotor.class, "intake_gear_motor");
        //shooter = hardwareMap.get(DcMotor.class, "shoot_motor");
        //wobble = hardwareMap.get(DcMotor.class, "wobble_motor");

        //shooter2 = hardwareMap.crservo.get("shoot_servo");
        //wobbleServo = hardwareMap.servo.get("wobble_servo");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armServo2 = hardwareMap.servo.get("arm2");

        bottomSense = hardwareMap.get(NormalizedColorSensor.class, "bottomColor");
        topSense = hardwareMap.get(NormalizedColorSensor.class, "topColor");

        // Color Sensor Setup
        float gain = 4;

        if (bottomSense instanceof SwitchableLight) {
            ((SwitchableLight) bottomSense).enableLight(true);
        }
        if (topSense instanceof SwitchableLight) {
            ((SwitchableLight) topSense).enableLight(true);
        }

        boolean a = false, b = false, c = false;

        final float[] hsvValues = new float[3];
        final float[] tHsvValues = new float[3];
        boolean start = false;

        double wobbleArmPos = 80;

        NormalizedRGBA colors = bottomSense.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        NormalizedRGBA tColors = topSense.getNormalizedColors();
        Color.colorToHSV(tColors.toColor(), tHsvValues);

        left1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left1Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // backwards: left +, right -
        // forwards: left -, right +

    // START OF AUTONOMOUS ********************
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        if (!start) {
            // at the beginning, put down arm and make sure claw is closed around wobble
            armMotor.setPower(0.5);
            sleep(2000);
            armServo2.setPosition(wobbleArmPos);
        sleep(1000);
        start = true;
        }

        // drive forward to the ring configuration
        encoderDrive(DRIVE_SPEED,  -18,  -18, 17.5, 17.5,5.0);  // S1: Forward 17 Inches with 5 Sec timeout - up to ring
        encoderDrive(DRIVE_SPEED,  -6,  6, -6, 6,5.0);  // S1: Strafe left

        // takes distance reading, letters (a,b,c) correspond to which path
        if ((((DistanceSensor) topSense).getDistance(DistanceUnit.CM)) < 10) {
            c = true;
            double inches = 85;
            encoderDrive(DRIVE_SPEED,  15,  -15, 15, -15,5.0);  // S1: Strafe left
            encoderDrive(DRIVE_SPEED,  -inches,  -inches, inches, inches,6.0);
            encoderDrive(DRIVE_SPEED,  -28,  28, -28, 28,5.0);  // S1: Strafe right
            // Open claw and pull up arm, releasing wobble goal
            armServo2.setPosition(0);
            sleep(1000);
            armMotor.setPower(-0.5);
            sleep(100);

            // drive to line
            encoderDrive(DRIVE_SPEED,  5,  5, -5, -5,5.0);
            encoderDrive(DRIVE_SPEED,  10,  -10, 10, -10,5.0);
            encoderDrive(DRIVE_SPEED,  15,  15, -15, -15,5.0);
        }

        // if a ring on bottom but none on top, has to be B
        // if no rings, must be A.
        if ((((DistanceSensor) bottomSense).getDistance(DistanceUnit.CM)) < 6) {
            if (!c) {
                encoderDrive(DRIVE_SPEED,  10,  -10, 10, -10,5.0);  // S1: Strafe left
                encoderDrive(DRIVE_SPEED,  -40,  -40, 40, 40,5.0);
                encoderDrive(DRIVE_SPEED,  -20,  20, -20, 20,5.0);  // Strafe right
                // open claw, pull up arm
                armServo2.setPosition(0);
                sleep(1000);
                armMotor.setPower(-0.5);
                sleep(100);

                // drive back to line
                encoderDrive(DRIVE_SPEED,  10,  10, -10, -10,5.0);
                b = true;
            } else {
                c = true;
            }
        } else if (!c) {
            a = true;
            encoderDrive(DRIVE_SPEED,  15,  -15, 15, -15,5.0);  // S1: Strafe left
            encoderDrive(DRIVE_SPEED,  -25,  -25, 25, 25,5.0);
            encoderDrive(DRIVE_SPEED,  -30,  30, -30, 30,5.0);  // S1: Strafe right
            // open claw, pull up arm
            armServo2.setPosition(0);
            sleep(1000);
            armMotor.setPower(-0.5);
            sleep(100);

            //pathing to line
            encoderDrive(DRIVE_SPEED,  5,  5, -5, -5,5.0);
            encoderDrive(DRIVE_SPEED,  30,  -30, 30, -30,5.0);  // S1: Strafe left
            encoderDrive(DRIVE_SPEED,  -18,  -18, 18, 18,5.0);
        }

        //encoderDrive(TURN_SPEED,   -12, -12, -12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        sleep(1000);     // pause for servos to move

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double left1Inches, double left2Inches,
                             double right1Inches, double right2Inches,
                             double timeoutS) {
        int newLeft1Target;
        int newLeft2Target;
        int newRight1Target;
        int newRight2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeft1Target = left1Drive.getCurrentPosition() + (int)(left1Inches * COUNTS_PER_INCH);
            newLeft2Target = left2Drive.getCurrentPosition() + (int)(left2Inches * COUNTS_PER_INCH);
            newRight1Target = right1Drive.getCurrentPosition() + (int)(right1Inches * COUNTS_PER_INCH);
            newRight2Target = right2Drive.getCurrentPosition() + (int)(right2Inches * COUNTS_PER_INCH);

            left1Drive.setTargetPosition(newLeft1Target);
            left2Drive.setTargetPosition(newLeft2Target);
            right1Drive.setTargetPosition(newRight1Target);
            right2Drive.setTargetPosition(newRight2Target);

            // Turn On RUN_TO_POSITION
            left1Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right1Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left1Drive.setPower(Math.abs(speed));
            left2Drive.setPower(Math.abs(speed));
            right1Drive.setPower(Math.abs(speed));
            right2Drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (left1Drive.isBusy() && right1Drive.isBusy()) && left2Drive.isBusy() && right2Drive.isBusy()) {

                // Display it for the driver.
                /*telemetry.addData("Path1",  "Running to %7d :%7d", newLeft1Target,  newRight1Target);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            left2Drive.getCurrentPosition(),
                                            right2Drive.getCurrentPosition());
                telemetry.update();*/
            }

            // Stop all motion;
            left1Drive.setPower(0);
            left2Drive.setPower(0);
            right1Drive.setPower(0);
            right2Drive.setPower(0);

            // Turn off RUN_TO_POSITION
            left1Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right1Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
