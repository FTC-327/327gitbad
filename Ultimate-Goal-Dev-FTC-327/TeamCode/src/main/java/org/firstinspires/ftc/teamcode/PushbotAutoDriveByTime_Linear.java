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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Pushbot: Auto Drive By Time", group="Auto")   // Drive to line, stop, turn/push wobble into B, extend arm
//@Disabled
public class PushbotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor left1Drive = null;      // 2: left front: GoBilda 5202
    private DcMotor left2Drive = null;      // 1: left back: GoBilda 5202
    private DcMotor right1Drive = null;     // 3: right front: GoBilda 5202
    private DcMotor right2Drive = null;     // 0: right back: GoBilda 5202

    private DcMotor intake = null;          // 0: Rev Hex motor Ultraplanetary
    private DcMotor intakeGear = null;      // 1: Rev Core hex motor
    private DcMotor shooter = null;         // 3: Rev Ultraplanetary
    private DcMotor wobble = null;          // 2: Rev Core hex

    private CRServo shooter2 = null;          // 0: servo
    private Servo wobbleServo = null;              // 1: servo

    private ColorSensor bottomSense = null;      // Color sensor
    private ColorSensor topSense = null;        // Color sensor

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

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

        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeGear = hardwareMap.get(DcMotor.class, "intake_gear_motor");
        shooter = hardwareMap.get(DcMotor.class, "shoot_motor");
        wobble = hardwareMap.get(DcMotor.class, "wobble_motor");

        shooter2 = hardwareMap.crservo.get("shoot_servo");
        wobbleServo = hardwareMap.servo.get("wobble_servo");

        bottomSense = hardwareMap.get(ColorSensor.class, "bottomColor");
        topSense = hardwareMap.get(ColorSensor.class, "topColor");



        //Update: ready to start
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 1.8 seconds
        left1Drive.setPower(-(FORWARD_SPEED-0.2));
        left2Drive.setPower(-(FORWARD_SPEED-0.2));
        right1Drive.setPower(FORWARD_SPEED-0.2);
        right2Drive.setPower(FORWARD_SPEED-0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2: stop for 0.2 seconds
        left1Drive.setPower(0);
        left2Drive.setPower(0);
        right1Drive.setPower(0);
        right2Drive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 1:  Drive forward for 1.8 seconds
        left1Drive.setPower(-TURN_SPEED);
        left2Drive.setPower(-TURN_SPEED);
        right1Drive.setPower(-TURN_SPEED);
        right2Drive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.9)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 2: stop for 0.2 seconds
        left1Drive.setPower(0);
        left2Drive.setPower(0);
        right1Drive.setPower(0);
        right2Drive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 1:  Drive forward for 1.8 seconds
        wobble.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        left1Drive.setPower(0);
        left2Drive.setPower(0);
        right1Drive.setPower(0);
        right2Drive.setPower(0);
        wobble.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100000);


        // Send telemetry message to signify robot waiting;
        /*telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        left1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left1Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                left1Drive.getCurrentPosition(),
                left2Drive.getCurrentPosition(),
                right1Drive.getCurrentPosition(),
                right2Drive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        encoderDrive(0.6, 20, 20,20, 20, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

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
            newLeft1Target = left1Drive.getCurrentPosition() + (int)(left1Inches * 386);
            newLeft2Target = left2Drive.getCurrentPosition() + (int)(left2Inches * 386);
            newRight1Target = right2Drive.getCurrentPosition() + (int)(right1Inches * 386);
            newRight2Target = right1Drive.getCurrentPosition() + (int)(right2Inches * 386);
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
                    (left1Drive.isBusy() && right2Drive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeft1Target,  newLeft2Target,
                        newRight1Target, newRight2Target);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        left1Drive.getCurrentPosition(),
                        right1Drive.getCurrentPosition());
                telemetry.update();
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

            //  sleep(250);   // optional pause after each move
        }*/
    }
}
