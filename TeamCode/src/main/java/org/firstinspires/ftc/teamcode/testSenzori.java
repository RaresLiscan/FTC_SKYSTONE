package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "Test Senzori")
@Disabled                            // Comment this out to add to the opmode list
public class testSenzori extends LinearOpMode {

    ColorSensor senzorDr;
    ColorSensor senzorSt;

    @Override
    public void runOpMode() {

        // get a reference to the color sensor.
        senzorDr = hardwareMap.get(ColorSensor.class, "senzorDr");
        senzorSt = hardwareMap.get(ColorSensor.class, "senzorSt");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();

        while (opModeIsActive()) {

            Color.RGBToHSV((int) (senzorSt.red() * SCALE_FACTOR),
                    (int) (senzorSt.green() * SCALE_FACTOR),
                    (int) (senzorSt.blue() * SCALE_FACTOR),
                    hsvValues);

            Color.RGBToHSV((int) (senzorDr.red() * SCALE_FACTOR),
                    (int) (senzorDr.green() * SCALE_FACTOR),
                    (int) (senzorDr.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Alpha", senzorSt.alpha());
            telemetry.addData("Red  ", senzorSt.red());
            telemetry.addData("Green", senzorSt.green());
            telemetry.addData("Blue ", senzorSt.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.addData("Alpha", senzorDr.alpha());
            telemetry.addData("Red  ", senzorDr.red());
            telemetry.addData("Green", senzorDr.green());
            telemetry.addData("Blue ", senzorDr.blue());
            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}