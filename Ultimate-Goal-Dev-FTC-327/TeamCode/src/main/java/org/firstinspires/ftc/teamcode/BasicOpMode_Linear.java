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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//testing 123

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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Control Hub
    private DcMotor left1Drive = null;      // 2: left front: GoBilda 5202
    private DcMotor left2Drive = null;      // 1: left back: GoBilda 5202
    private DcMotor right1Drive = null;     // 3: right front: GoBilda 5202
    private DcMotor right2Drive = null;     // 0: right back: GoBilda 5202

    // Expansion Hub
    private DcMotor intake = null;          // 0: Rev Hex motor Ultraplanetary
    private DcMotor intakeGear = null;      // 1: Rev Core hex motor
    private DcMotor shooter = null;         // 3: Rev Ultraplanetary
    private DcMotor armMotor = null;          // 2: Rev Core hex

    private CRServo shooter2 = null;          // 0: servo
    private Servo armServo = null;         // 1: servo

    private NormalizedColorSensor bottomSense = null;      // Color sensor
    private NormalizedColorSensor topSense = null;        // Color sensor

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left1Drive  = hardwareMap.get(DcMotor.class, "left_front");
        left2Drive  = hardwareMap.get(DcMotor.class, "left_back");
        right1Drive = hardwareMap.get(DcMotor.class, "right_front");
        right2Drive = hardwareMap.get(DcMotor.class, "right_back");

        intake      = hardwareMap.get(DcMotor.class, "intake_motor");
        //intakeGear  = hardwareMap.get(DcMotor.class, "intake_gear_motor");
        shooter     = hardwareMap.get(DcMotor.class, "shoot_motor");
        armMotor      = hardwareMap.get(DcMotor.class, "arm");

        shooter2    = hardwareMap.crservo.get("shoot_servo");
        armServo = hardwareMap.servo.get("arm2");

        bottomSense = hardwareMap.get(NormalizedColorSensor.class, "bottomColor");
        topSense = hardwareMap.get(NormalizedColorSensor.class, "topColor");

        //Foward = cw, reverse = ccw
         left1Drive.setDirection(DcMotor.Direction.FORWARD);
        // left2Drive.setDirection(DcMotor.Direction.REVERSE); // ONLY reverse for test bot
         left2Drive.setDirection(DcMotor.Direction.FORWARD);
         right1Drive.setDirection(DcMotor.Direction.REVERSE);
         right2Drive.setDirection(DcMotor.Direction.REVERSE);

         intake.setDirection(DcMotor.Direction.FORWARD);
         //intakeGear.setDirection(DcMotor.Direction.FORWARD);
         shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double left1Power;
            double left2Power;
            double right1Power;
            double right2Power;

            double intakePower;
            double intakeGearPower;
            double shooterPower;
            double servoPower;
            double wobblePower;

            int wobbleArmTarget;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
               double drive = -(gamepad1.left_stick_y);
               double turn = gamepad1.right_stick_x * 1.5;
               double rightStrafe  =  gamepad1.left_trigger;
               double leftStrafe = gamepad1.right_trigger;
               boolean trigger = gamepad1.right_bumper;
               boolean armUp = gamepad2.left_bumper;
               boolean armDown = gamepad2.right_bumper;
               boolean slow = false;
               boolean slowMode2 = false;

               slow = trigger;

            // Used for intake testing: change to triggers after done
            // also: might be nice to have one button for both motors
            double intake1 = gamepad2.left_stick_y;
            //double intake2 = gamepad2.right_stick_y;
            double shoot = gamepad2.right_stick_y;
            boolean armSlow = gamepad2.left_bumper;     // slow mode for arm
            double wobbleArm = gamepad2.right_trigger;
            double negWobbleArm = gamepad2.left_trigger;
            // WOBBLE GRIP is A for close and B for open.
            // SHOOTER SERVO is Y for return up.
            double wobbleGripPos = 0.5;
            double armPos = 100;
            double shooterServoPos = 10;

            //double left_back = gamepad1.left_trigger; // 1      ltrigger
            //double left_front = gamepad1. left_stick_y; // 2    lstick
            //double right_back = gamepad1.right_trigger; // 0    rtrigger
            //double right_front = gamepad1.right_stick_y; // 3   rstick

            //Gamepad 1: drive
            //Gamepad 2: operations (intake = left stick, shoot = right stick)

            //left1Power    = Range.clip(drive, -1.0, 1.0) ;
            //left2Power    = Range.clip(drive, -1.0, 1.0) ;
            //right1Power   = Range.clip(drive, -1.0, 1.0) ;
            //right2Power   = Range.clip(drive, -1.0, 1.0) ;

            double strafe = leftStrafe - rightStrafe;

                left1Power = Range.clip((-drive) - strafe - turn, -1.0, 1.0);
                left2Power = Range.clip((-drive) + strafe - turn, -1.0, 1.0);
                right1Power = Range.clip((-drive) + strafe + turn, -1.0, 1.0);
                right2Power = Range.clip((-drive) - strafe + turn, -1.0, 1.0);

            intakePower = Range.clip(intake1, -1.0, 1.0);
            intakeGearPower = Range.clip(intake1, -1.0, 1.0);
            shooterPower = Range.clip(shoot, 0.0, 0.85);
            servoPower = Range.clip(shoot, 0.0, 1.0);

            wobblePower = Range.clip((-negWobbleArm)+wobbleArm, -1.0, 1.0);
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            if (slow) {
                left1Drive.setPower(left1Power/3);
                left2Drive.setPower(left2Power/3);
                right1Drive.setPower(right1Power/3);
                right2Drive.setPower(right2Power/3);
            } else {
                left1Drive.setPower(left1Power*0.7);
                left2Drive.setPower(left2Power*0.7);
                right1Drive.setPower(right1Power*0.7);
                right2Drive.setPower(right2Power*0.7);
            }

            intake.setPower(intakePower);
            //intakeGear.setPower(intakeGearPower);
            shooter.setPower(shooterPower);

            if (gamepad2.y && shooterServoPos < 100) shooterServoPos += 1;
            if (gamepad2.x && shooterServoPos < 100) shooterServoPos -= 1;
            if (!gamepad2.y && !gamepad2.x) {
                shooter2.setPower(0);
            } else if (gamepad2.y && !gamepad2.x){
                shooter2.setPower(shooterServoPos);
            } else if (gamepad2.x && !gamepad2.y){
                shooter2.setPower(-shooterServoPos);
            } else {
                shooter2.setPower(0);
            }
            if (armUp && armPos < 100) armPos += 1;
            if (armDown && armPos < 100) armPos -= 1;
            if (!armUp && !armDown) {
                armServo.setPosition(0);
            }


            if (armSlow && slowMode2) {
                slowMode2 = false;
            } else if (armSlow && !slowMode2) {
                slowMode2 = true;
            }

            if (slowMode2) {
                armMotor.setPower(wobblePower/4);
            } else {
                armMotor.setPower(wobblePower);
            }

            //A for close, B for open
            if (gamepad2.a && (wobbleGripPos > -5)) wobbleGripPos -= 1;
            if (gamepad2.b && (wobbleGripPos < 30)) wobbleGripPos += 1;
            armServo.setPosition(wobbleGripPos);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left_front (%.2f), left_back (%.2f), right_front (%.2f), right_back, (%.2f), " +
                            "intake_motor (%.2f), shoot_motor (%.2f), shoot_servo (%.2f), slowMode (%.2b" +
                            ")",
                            left1Power, left2Power, right1Power, right2Power,       // Drive Powers
                            intakePower, shooterPower, servoPower, slow); // Extra Motor Powers
            telemetry.update();

            //Activates the intake motor when pulling the left trigger
            /*double leftTrigger = gamepad1.left_trigger;
            if (leftTrigger > 0) {
                intake.setPower(1);
            }*/
        }
    }
}