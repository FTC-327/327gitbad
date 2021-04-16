package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ColorAuto", group="Linear Opmode")
public class AutoColor extends LinearOpMode {

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

    private NormalizedColorSensor bottomSense = null;      // Color sensor
    private NormalizedColorSensor topSense = null;        // Color sensor

    private ElapsedTime runtime = new ElapsedTime();
    View relativeLayout;

    @Override
    public void runOpMode() {

        try {
            runSample(); // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    protected void runSample() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //left1Drive = hardwareMap.get(DcMotor.class, "left_front");
        //left2Drive = hardwareMap.get(DcMotor.class, "left_back");
        //right1Drive = hardwareMap.get(DcMotor.class, "right_front");
        //right2Drive = hardwareMap.get(DcMotor.class, "right_back");

        //intake = hardwareMap.get(DcMotor.class, "intake_motor");
        //intakeGear = hardwareMap.get(DcMotor.class, "intake_gear_motor");
        shooter = hardwareMap.get(DcMotor.class, "shoot_motor");
        //wobble = hardwareMap.get(DcMotor.class, "wobble_motor");

        shooter2 = hardwareMap.crservo.get("shoot_servo");
        wobbleServo = hardwareMap.servo.get("wobble_servo");

        bottomSense = hardwareMap.get(NormalizedColorSensor.class, "bottomColor");
        topSense = hardwareMap.get(NormalizedColorSensor.class, "topColor");

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        float gain = 4;

        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        if (bottomSense instanceof SwitchableLight) {
            ((SwitchableLight)bottomSense).enableLight(true);
        }
        if (topSense instanceof SwitchableLight) {
            ((SwitchableLight)topSense).enableLight(true);
        }

        final float[] hsvValues = new float[3];
        final float[] tHsvValues = new float[3];
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
                double shootPower;
                double servoPower;

                boolean a, b, c;

            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }
            // Show the gain value via telemetry
            telemetry.addData("Gain", gain);

            topSense.setGain(gain);
            bottomSense.setGain(gain);
            // Check the status of the X button on the gamepad
            xButtonCurrentlyPressed = gamepad1.x;

            // If the button state is different than what it was, then act
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                // If the button is (now) down, then toggle the light
                if (xButtonCurrentlyPressed) {
                    if (bottomSense instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight)bottomSense;
                        light.enableLight(!light.isLightOn());
                    }
                    if (topSense instanceof  SwitchableLight) {
                        SwitchableLight light2 = (SwitchableLight)topSense;
                        light2.enableLight(!light2.isLightOn());
                    }
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            NormalizedRGBA colors = bottomSense.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            // Show the elapsed game time and wheel power.
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            double bottomDis = (((DistanceSensor) bottomSense).getDistance(DistanceUnit.CM));

            NormalizedRGBA tColors = topSense.getNormalizedColors();

            Color.colorToHSV(tColors.toColor(), tHsvValues);

            // Show the elapsed game time and wheel power.
            telemetry.addLine()
                    .addData("Red", "%.3f", tColors.red)
                    .addData("Green", "%.3f", tColors.green)
                    .addData("Blue", "%.3f", tColors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", tHsvValues[0])
                    .addData("Saturation", "%.3f", tHsvValues[1])
                    .addData("Value", "%.3f", tHsvValues[2]);
            telemetry.addData("Alpha", "%.3f", tColors.alpha);

            double redB = (Math.pow((0.025*(bottomDis)), -1))/1000;
            telemetry.addData("REDB", "%.3f", redB);
// *****************END OF MATH***************************
                // if a ring on top, has to be C configuration
                if ((tColors.red >= 0.01 && tColors.red <= 0.02) && (tColors.green >= 0.01 && tColors.green <= 0.02)) {
                    servoPower = 1;
                    c = true;
                } else {
                    servoPower = 0;
                    c = false;
                }

                // if a ring on bottom but none on top, has to be B
                // if no rings, must be A.
                if ((colors.red >= 0.01 && colors.red <= 0.02) && (colors.green >= 0.01 && colors.green <= 0.02)) {
                    shootPower = 1;
                    if (!c) {
                        b = true;
                    } else {
                        c = true;
                    }
                } else {
                    shootPower = 0;
                    a = true;
                }

                shooter.setPower(shootPower);
                shooter2.setPower(servoPower);


            if (topSense instanceof DistanceSensor) {
                telemetry.addData("Top Distance (cm)", "%.3f", ((DistanceSensor) topSense).getDistance(DistanceUnit.CM));
            }
            if (bottomSense instanceof DistanceSensor) {
                telemetry.addData("Top Distance (cm)", "%.3f", ((DistanceSensor) bottomSense).getDistance(DistanceUnit.CM));
            }

            telemetry.update();

            // Change the Robot Controller's background color to match the color detected by the color sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                      }
            });
        }

    }
}
