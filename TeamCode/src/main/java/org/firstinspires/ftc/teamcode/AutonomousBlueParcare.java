package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Albastru completa perete")
@Disabled
public class AutonomousBlueParcare extends LinearOpMode {

    private RobotMap robot = null;
    double colorError = 18;
    double conversieCmTick = 0;


    boolean checkSensor(double a, double b, double c) {
        if (Math.abs(a - b) > colorError || Math.abs(a - c) > colorError || Math.abs(b - c) > colorError) return false;
        return true;
    }


    void rotateFundation() {
        robot.macaraStangaEncoder(-95, 0.8, 2);
        robot.macaraDreaptaEncoder(-70, 0.8, 2);

        robot.ridicareBratStanga.setPower(0.8);
        robot.ridicareBratDreapta.setPower(0.8);
        sleep(1000);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.rotateConstantSpeed(15, 1, 1);

        robot.runUsingEncoders(-robot.conversieCmToTick(440), 1, 2);//1800

        robot.rotateConstantSpeed(63, 1, 3);

        robot.runUsingEncoders(robot.conversieCmToTick(280), 0.5, 2);//500

        robot.ridicareBratDreapta.setPower(-0.85);
        robot.ridicareBratStanga.setPower(-0.85);

        robot.runUsingEncoders(-robot.conversieCmToTick(100), 1, 3);

        robot.strafe(-950, 0.6, 2);

        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        double correction = robot.maintainAngle();
        robot.rotateEncoders(robot.conversieDegreesToTicks(correction + 3), 1, 1);

        robot.runUsingEncoders(-robot.conversieCmToTick(950), 1, 2);


//
//        robot.strafe(300, 1, 1);
//
//        robot.runUsingEncoders(-robot.conversieCmToTick(800), 1, 3);//3000
////
//        robot.ghearaStanga.setPosition(0.23);
//        sleep(1000);
    }


    void skystoneLeft() {
        telemetry.addData("Position: ", "left");
        telemetry.update();

        robot.runUsingEncoders(150, 0.8, 3);
        robot.ghearaStanga.setPosition(0);
        sleep(600);

        robot.ridicareBratDreapta.setPower(-0.85);
        robot.ridicareBratStanga.setPower(-0.85);
        robot.runUsingEncoders(-robot.conversieCmToTick(200), 0.7, 3);
        robot.ridicareBratDreapta.setPower(0);
        robot.ridicareBratStanga.setPower(0);

//        robot.rotate(83, 1, 2);
        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        // robot.runUsingEncoders(7300, 1, 9);
        // robot.prevAngle = robot.getAngle();
        robot.runUsingEncoders(robot.conversieCmToTick(2230), 1, 9);

        // double correction = robot.maintainAngle();
        // robot.rotateEncoders(robot.convertDegreesToTicks(correction + 2), 1, 1);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
//        robot.rotate(-84, 1, 3);
        robot.rotateEncoders(-robot.conversieDegreesToTicks(89), 1, 2);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 190), 0.3, 4);

        robot.ghearaStanga.setPosition(0.5);
        rotateFundation();
    }

    void skystoneRight() {
        telemetry.addData("Position: ", "right");
        telemetry.update();
        robot.runUsingEncoders(robot.conversieCmToTick(60),0.4, 3);
        robot.ghearaDreapta.setPosition(1);
        sleep(600);
        robot.ridicareBratDreapta.setPower(-0.85);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(600);
        robot.ridicareBratDreapta.setPower(0);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-robot.conversieCmToTick(200), 0.8, 3);

        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        robot.runUsingEncodersCorrection(robot.conversieCmToTick(2230), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.2);
        }

        robot.rotateEncoders(-robot.conversieDegreesToTicks(89), 1, 2);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 190), 0.3, 4);

        robot.ghearaDreapta.setPosition(0.5);
        rotateFundation();
    }

    void noSkystone() {

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 95) {
            robot.forward(-0.25);
        }

        robot.strafe(550, 0.6, 3);//1000
        double correction = robot.maintainAngle();
        robot.rotateEncoders(robot.conversieDegreesToTicks(correction + 3), 1, 2);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.25);
        }

        robot.runUsingEncoders(150, 0.6, 3);
        robot.ghearaDreapta.setPosition(1);
        sleep(600);

        robot.ridicareBratDreapta.setPower(-0.85);
        robot.ridicareBratStanga.setPower(-0.85);
        robot.runUsingEncoders(-400, 0.8, 3);
        robot.ridicareBratDreapta.setPower(0);
        robot.ridicareBratStanga.setPower(0);

        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        robot.runUsingEncoders(robot.conversieCmToTick(2230), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.4);
        }
        robot.rotateEncoders(-robot.conversieDegreesToTicks(89), 1, 2);

        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 190), 0.3, 4);

        robot.ghearaDreapta.setPosition(0.5);
        rotateFundation();

    }




    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        // robot.zeroPowerBeh();
        waitForStart();

        telemetry.addData("Status: ", "ready");
        telemetry.update();

        if (opModeIsActive()) {

            robot.senzorStanga.enableLed(true);
            robot.senzorDreapta.enableLed(true);
            robot.ghearaDreapta.setPosition(0.5);
            robot.ghearaStanga.setPosition(0.5);

            robot.ridicareBratDreapta.setPower(0.5);
            robot.ridicareBratStanga.setPower(0.5);

            robot.scripeteDreapta.setPower(0.3);
            robot.scripeteStanga.setPower(0.3);

            robot.runUsingEncoders(robot.conversieCmToTick(620), 0.4, 4);

            robot.ridicareBratDreapta.setPower(0);
            robot.scripeteDreapta.setPower(0);
            robot.scripeteStanga.setPower(0);
            robot.ridicareBratStanga.setPower(0);


            // robot.runUsingEncoders(2500, 0.6, 4);//2550


            //  while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 45) {
            //      robot.forward(-0.2);
            //      // telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            //      // telemetry.update();
            //  }

            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
                robot.forward(0.25);
                // telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
                // telemetry.update();
            }

            robot.stopDriving();


            sleep(500);

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