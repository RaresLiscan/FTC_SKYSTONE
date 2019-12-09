/* Copyright (c) 2019 FIRST. All rights reserved.
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

//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Tensorflow Object Detection")
@Disabled
public class TensorflowAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    private void basculare (int ticks, double power, DcMotor motor) {
        if (ticks < 0) motor.setPower(-power);
        else motor.setPower(power);
    }

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQxsYqj/////AAABmY6ANHQE609wt3JyeoRyc4N7k4HDzWHqdTJWnAgd+scnAqVM7dFeQQHVYNemssNx75C99yCoq9fEymeC7SY8N6GJWPz6VJh1vN2fyo787yiIQlxUNPkbRfopwMf1bjxDig33D1xe0y+n6uqLj3Th3mm+jkCQK+NYJZaPU1F+e7sg/FNxQ7R5tDCcSKW6JStsNHbfmF6nuzwLE9kGri4a4Y33WjOvmPtbst39rx7TSag4kpvmsjkxsZu2AMR53cAe+unbHYmnnOb6hL+z90fgPYAXCYMqGDpvye9jB/TgC+gupyjB/PiGLKzzd/NXV4Mn+N6jzPk/lGz1uD10c0MSIOGcRxw3xU47VDM57827a8uM";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private RobotMap robot = null;

    private void runForSkystone (Recognition recognition) {
        double l = recognition.getLeft(), r = recognition.getRight();
        double t = recognition.getTop(), b = recognition.getBottom();
        double power = 0.3, strafePower = 0.6;

        if (Math.abs(t - b) > 650) {
            if (l >= 0 && r <= 1300) {
                robot.runMotors(-power, -power, power, power);
                telemetry.addData("Status: ", " merem stanga si ciordim");
            }
            else if (l < 0) {
                robot.runMotors(-strafePower, strafePower, -strafePower, strafePower);
                telemetry.addData("Status: ", " merem stanga ca n-avem niciun drept");
            }
            else if (r > 1300) {
                robot.runMotors(strafePower, -strafePower, strafePower, -strafePower);
                telemetry.addData("Status: ", " merem dreapta sa fim stangi");
            }
        }
        else {
            if (l >= 0 && r <= 1300) {
                robot.strafe(1400, strafePower, 3);
                robot.runUsingEncoders(800, power, 3);
                //Il prindem si ne-am dus
                telemetry.addData("Status: ", " merem stanga si ciordim");
            }
            else if (l < 0) {
                robot.runMotors(-strafePower, strafePower, -strafePower, strafePower);
                telemetry.addData("Status: ", " merem stanga ca n-avem niciun drept");
            }
            else if (r > 1300) {
                robot.runMotors(strafePower, -strafePower, strafePower, -strafePower);
                telemetry.addData("Status: ", " merem dreapta sa fim stangi");
            }
        }
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        CameraDevice.getInstance().setFlashTorchMode(true);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        robot = new RobotMap(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel() == "Skystone") {
//                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                        recognition.getLeft(), recognition.getTop());
//                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                        recognition.getRight(), recognition.getBottom());
                                telemetry.addData("Top: ", recognition.getTop());
                                telemetry.addData("Botton: ", recognition.getBottom());
                                telemetry.addData("Left: ", recognition.getLeft());
                                telemetry.addData("Right: ", recognition.getRight());
                                double l = recognition.getLeft(), r = recognition.getRight();
                                double t = recognition.getTop(), b = recognition.getBottom();

                                if (Math.abs(t - b) > 650) {
                                    telemetry.addData("Status: ", " hai ca ne-am apropiat");
                                    if (l >= 0 && r <= 1300) {
                                        telemetry.addData("Status: ", " merem stanga si ciordim");
                                    }
                                    else if (l < 0) {
                                        telemetry.addData("Status: ", " merem stanga ca n-avem niciun drept");
                                    }
                                    else if (r > 1300) {
                                        telemetry.addData("Status: ", " merem dreapta sa fim stangi");
                                    }
                                }
                                else {
                                    telemetry.addData("Status: ", " da-i in fata ticut");
                                    if (l >= 0 && r <= 1300) {
                                        telemetry.addData("Status: ", " merem stanga si ciordim");
                                    }
                                    else if (l < 0) {
                                        telemetry.addData("Status: ", " merem stanga ca n-avem niciun drept");
                                    }
                                    else if (r > 1300) {
                                        telemetry.addData("Status: ", " merem dreapta sa fim stangi");
                                    }
                                }
                            }
                        }
                        telemetry.update();
                    }
                }

                /** GAMEPAD 1 */
                //Miscarea sasiului
                double rotate = -gamepad1.right_stick_x * 0.45;
                double strafe =  gamepad1.left_stick_x * 0.7;
                double forward = gamepad1.left_stick_y * 0.45;

                double sS = -strafe - forward - rotate;
                double dF =  strafe + forward - rotate;
                double sF =  strafe - forward - rotate;
                double dS = -strafe + forward - rotate;

                robot.stangaSpate.setPower(sS);
                robot.dreaptaFata.setPower(dF);
                robot.stangaFata.setPower(sF);
                robot.dreaptaSpate.setPower(dS);



                //Ridicarea scripetelui
                robot.scripeteDreapta.setPower(gamepad1.right_stick_y * 0.7);


                //Prinderea mineralului
                if (gamepad1.right_trigger != 0) {
                    robot.ghearaDreapta.setPosition(1);
                }
                if (gamepad1.right_bumper) {
                    robot.ghearaDreapta.setPosition(0.5);
                }

                //Eliberare capstone
                if (gamepad1.left_trigger != 0) {
                    robot.ghearaStanga.setPosition(0);
                }
                if (gamepad1.left_bumper) {
                    robot.ghearaStanga.setPosition(0.5);
                }


                //Bascularea bratului
                double raisePower = 0.4;
                int ticks = 1440;
                //Pentru bratul din dreapta
                if (gamepad1.a) {
                    basculare(-ticks, raisePower, robot.ridicareBratDreapta);
                }
                else if (gamepad1.y) {
                    basculare(ticks, raisePower, robot.ridicareBratDreapta);
                }
                else robot.ridicareBratDreapta.setPower(0);

                //Pentru bratul din stanga
                if (gamepad1.dpad_up) {
                    basculare(ticks, raisePower, robot.ridicareBratStanga);
                }
                else if (gamepad1.dpad_down) {
                    basculare(-ticks, raisePower, robot.ridicareBratStanga);
                }
                else robot.ridicareBratStanga.setPower(0);


                /** GAMEPAD 2 */


                //Ridicare scripete dreapta
                robot.scripeteDreapta.setPower(gamepad2.right_stick_y * 0.7);
                //Ridicare scripete stanga
                robot.scripeteStanga.setPower(gamepad2.left_stick_y * 0.7);


                //Bascularea bratului
                double powerRise = 0.4;
                //Pentru bratul din dreapta
                if (gamepad2.a) {
                    basculare(-ticks, powerRise, robot.ridicareBratDreapta);
                }
                else if (gamepad2.y) {
                    basculare(ticks, powerRise, robot.ridicareBratDreapta);
                }
                else robot.ridicareBratDreapta.setPower(0);
                //Pentru bratul din stanga
                if (gamepad2.dpad_up) {
                    basculare(ticks, powerRise, robot.ridicareBratStanga);
                }
                else if (gamepad2.dpad_down) {
                    basculare(-ticks, powerRise, robot.ridicareBratStanga);
                }
                else robot.ridicareBratStanga.setPower(0);

                //Prinderea mineralului pentru bratul din dreapta
                if (gamepad2.right_bumper) {
                    robot.ghearaDreapta.setPosition(0.5);
                }
                else if (gamepad2.right_trigger != 0) {
                    robot.ghearaDreapta.setPosition(1);
                }
                //Prinderea de capstone
                if (gamepad2.left_trigger != 0) {
                    robot.ghearaStanga.setPosition(0);
                }
                else if (gamepad2.left_bumper) {
                    robot.ghearaStanga.setPosition(0.5);
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
