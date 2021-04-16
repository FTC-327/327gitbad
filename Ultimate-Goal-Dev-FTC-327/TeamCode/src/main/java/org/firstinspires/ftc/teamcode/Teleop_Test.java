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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="WobbleAuto", group="Linear Opmode")
public class Teleop_Test extends LinearOpMode {

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

        bottomSense = hardwareMap.colorSensor.get("bottom_color");
        topSense = hardwareMap.colorSensor.get("top_color");

        //Update: ready to start
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        // Step 1:  Drive forward for 2 seconds to rings
        bottomSense.enableLed(true);
        topSense.enableLed(true);

        left1Drive.setPower(-FORWARD_SPEED);
        left2Drive.setPower(-FORWARD_SPEED);
        right1Drive.setPower(FORWARD_SPEED);
        right2Drive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
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

        // Step 3:  Strafe right  seconds to rings
        left1Drive.setPower(-FORWARD_SPEED);
        left2Drive.setPower(FORWARD_SPEED);
        right1Drive.setPower(FORWARD_SPEED);
        right2Drive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        left1Drive.setPower(0);
        left2Drive.setPower(0);
        right1Drive.setPower(0);
        right2Drive.setPower(0);
        bottomSense.enableLed(false);
        topSense.enableLed(false);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100000);

    }
}
