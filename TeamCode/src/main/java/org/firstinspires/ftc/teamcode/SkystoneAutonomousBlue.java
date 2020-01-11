package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Detect Skystone Albastru")
public class SkystoneAutonomousBlue extends LinearOpMode {

    RobotMap robot = null;
    double colorError = 20;

    boolean checkSensor(double a, double b, double c) {
        if (Math.abs(a - b) > colorError || Math.abs(a - c) > colorError || Math.abs(b - c) > colorError) return false;
        return true;
    }


    void rotateFundation() {
        robot.ridicareBratStanga.setPower(0.8);
        robot.ridicareBratDreapta.setPower(0.8);
        sleep(1000);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.rotate(15, 0.4, 5);

        robot.runUsingEncoders(-1750, 0.4, 5);//1800

        robot.rotate(70, 0.4, 5);

        robot.ridicareBratDreapta.setPower(-0.8);
        robot.ridicareBratStanga.setPower(-0.8);
        sleep(1000);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.runUsingEncoders(900, 0.4, 5);//500

        robot.runUsingEncoders(-3650, 0.7, 5);//3000

        robot.rotate(-78, 0.7, 5);

        double correction = robot.checkDirection();
        robot.rotate((int) correction, 0.2, 2);

        sleep(1000);
    }


    void skystoneLeft() {
        telemetry.addData("Position: ", "left");
        telemetry.update();
        robot.strafe(-1000, 0.65, 3);
        double correction = robot.checkDirection();
        robot.rotate((int) correction, 0.3, 2);
        robot.runUsingEncoders(400, 0.4, 3);
        robot.ghearaDreapta.setPosition(1);
        sleep(600);
        robot.ridicareBratDreapta.setPower(-0.85);
        sleep(700);
        robot.ridicareBratDreapta.setPower(0);
        robot.runUsingEncoders(-600, 0.5, 3);
        robot.rotate(78, 0.7, 2);
        robot.runUsingEncoders(7450, 0.8, 9);
        robot.rotate(-79, 0.7, 3);
        robot.runUsingEncoders(680, 0.5, 3);//550
        robot.ghearaDreapta.setPosition(0.5);
        robot.macaraDreaptaEncoder(-70, 0.7, 2);
        rotateFundation();
    }

    void skystoneRight() {
        telemetry.addData("Position: ", "right");
        telemetry.update();
        robot.runUsingEncoders(300,0.4, 3);
        robot.ghearaDreapta.setPosition(1);
        sleep(600);
        robot.ridicareBratDreapta.setPower(-0.85);
        sleep(700);
        robot.ridicareBratDreapta.setPower(0);
        robot.runUsingEncoders(-600, 0.5, 3);
        robot.rotate(78, 0.7, 2);
        robot.runUsingEncoders(8000, 0.8, 9);
        robot.rotate(-79, 0.7, 3);
        robot.runUsingEncoders(680, 0.5, 3);//550
        robot.ghearaDreapta.setPosition(0.5);
        robot.macaraDreaptaEncoder(-70, 0.7, 2);
        rotateFundation();
    }

    void noSkystone() {
        telemetry.addData("Position: ", "no skystone");
        telemetry.update();
        sleep(5000);
//        robot.runUsingEncoders(-500, 0.4, 3);
//        robot.runUsingEncoders(-500, 0.6, 3);
//        robot.strafe(2500, 0.6, 3);
//        robot.scripeteDreapta.setPower(0.85);
//        sleep(600);
//        robot.scripeteDreapta.setPower(0);
//        robot.runUsingEncoders(1000, 0.6, 3);
//        robot.ghearaDreapta.setPosition(1);
//        sleep(600);
//        robot.ridicareBratDreapta.setPower(-0.85);
//        sleep(600);
//        robot.ridicareBratDreapta.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            robot.senzorStanga.enableLed(true);
            robot.senzorDreapta.enableLed(true);
            robot.ghearaDreapta.setPosition(0.5);
            robot.ridicareBratDreapta.setPower(0.85);
            robot.scripeteDreapta.setPower(0.85);
            sleep(600);
            robot.ridicareBratDreapta.setPower(0);
            robot.scripeteDreapta.setPower(0);
            robot.runUsingEncoders(2550, 0.6, 4);//2550
            sleep(1000);

            double rStanga = robot.senzorStanga.red();
            double gStanga = robot.senzorStanga.green();
            double bStanga = robot.senzorStanga.blue();

            double rDreapta = robot.senzorDreapta.red();
            double gDreapta = robot.senzorDreapta.green();
            double bDreapta = robot.senzorDreapta.blue();

            robot.senzorDreapta.enableLed(false);
            robot.senzorStanga.enableLed(false);
            if (checkSensor(rStanga, gStanga, bStanga)) {
                skystoneLeft();
            }
            else if (checkSensor(rDreapta, gDreapta, bDreapta)) {
                skystoneRight();
            }
            else {
                noSkystone();
            }

        }
    }
}
