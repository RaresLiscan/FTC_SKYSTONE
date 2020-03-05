package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Rosu2ss")
@Disabled
public class Rosu2ss extends LinearOpMode {

    private RobotMap robot = null;
    double colorError = 18;
    double conversieCmTick = 0;

    boolean checkSensor(double a, double b, double c) {
        if (Math.abs(a - b) > colorError || Math.abs(a - c) > colorError || Math.abs(b - c) > colorError) return false;
        return true;
    }



    void skystoneRight() {
        telemetry.addData("Position: ", "right");
        telemetry.update();

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.25);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }
        robot.stopDriving();

//        robot.strafe(660, 0.60, 3);//1000
        double correction = robot.checkDirection();
//        robot.rotateConstantSpeed((int) correction, 0.35, 2);
        robot.rotateEncoders(robot.conversieDegreesToTicks(correction), 0.6, 1);



        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.25);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }

        robot.stopDriving();

        robot.runUsingEncoders(150, 0.75, 3);

        robot.ghearaDreapta.setPosition(1);
        sleep(600);

        robot.ridicareBratStanga.setPower(-0.85);
        robot.ridicareBratDreapta.setPower(-0.85);
        sleep(700);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.runUsingEncoders(-200, 0.8, 3);
//        robot.rotate(-83, 1, 2);//-79
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);
        robot.runUsingEncoders(2800, 0.7, 4);


        robot.ridicareBratDreapta.setPower(0.85);
        sleep(500);
        robot.ridicareBratDreapta.setPower(0);


        robot.ghearaDreapta.setPosition(0.5);
        sleep(600);
        robot.runUsingEncoders(-150, 0.6, 2);
        robot.ridicareBratDreapta.setPower(-0.8);
        sleep(500);
        robot.ridicareBratDreapta.setPower(0);

        robot.runUsingEncoders(-3335, 0.7, 6);//3350

        robot.ridicareBratStanga.setPower(0.8);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);


        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 1);

        while (robot.senzorDistanta.getDistance(DistanceUnit.CM) > 1) {
            robot.forward(0.25);
        }
        robot.stopDriving();
        robot.runUsingEncoders(150, 0.7, 1);
        robot.ghearaStanga.setPosition(0);
        sleep(500);
        robot.ridicareBratStanga.setPower(-0.85);
        robot.runUsingEncoders(-200, 0.7, 1);
        sleep(100);
        robot.ridicareBratStanga.setPower(0);

        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 1);

        robot.runUsingEncoders(3100, 0.7, 4);

        robot.ridicareBratStanga.setPower(0.8);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);
        robot.ghearaStanga.setPosition(0.5);
        sleep(500);

        robot.runUsingEncoders(-150, 0.7, 1);
        robot.ridicareBratStanga.setPower(-0.8);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);

        robot.runUsingEncoders(-350, 1, 1);

//        robot.ridicareBratStanga.setPower(0);
//        sleep(1000);
/*
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.rotate(84, 1, 3);
        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 100), 0.3, 4);
        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-95, 0.7, 2);
*/
    }

    void skystoneLeft() {
        telemetry.addData("Position: ", "left");
        telemetry.update();
        robot.runUsingEncoders(robot.conversieCmToTick(60),0.8, 3);
        robot.ghearaStanga.setPosition(0);
        sleep(300);
        robot.ridicareBratStanga.setPower(-1);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-robot.conversieCmToTick(200), 1, 3);
//        robot.rotate(-82, 1, 2);//
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 0.7, 2);
        robot.runUsingEncoders(robot.conversieCmToTick(2230), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }
//        robot.rotate(85, 1, 3);//
        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 0.7, 2);
        conversieCmTick =robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 160), 0.35, 5);
        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-95, 1, 2);
    }

    void noSkystone() {
        telemetry.addData("Position: ", "no skystone");
        telemetry.update();

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 75) {
            robot.forward(-0.25);
        }

        robot.strafe(-750, 0.6, 3);//-800, 1
        double correction = robot.maintainAngle();
        robot.rotateEncoders(robot.conversieDegreesToTicks(correction - 3), 1, 1);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.25);
        }


        robot.runUsingEncoders(150, 0.8, 3);//
        robot.ghearaStanga.setPosition(0);
        sleep(500);
        robot.ridicareBratStanga.setPower(-0.85);
        robot.ridicareBratDreapta.setPower(-0.85);
        sleep(600);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);
        robot.runUsingEncoders(-200, 0.5, 3);
//        robot.rotate(-83, 1, 2);
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);
        robot.runUsingEncoders(3150, 0.7, 4);
        //sleep(500);
        robot.ridicareBratStanga.setPower(0.8);
        sleep(600);
        robot.ghearaStanga.setPosition(0.5);
        sleep(500);
        robot.runUsingEncoders(-200, 0.5, 2);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-1790, 0.7, 4);

        robot.ridicareBratStanga.setPower(0.8);
        sleep(600);
        robot.ridicareBratStanga.setPower(0);

        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        while (robot.senzorDistanta.getDistance(DistanceUnit.CM) > 1) {
            robot.forward(0.25);
        }
        robot.runUsingEncoders(150, 0.8, 3);



        robot.ghearaStanga.setPosition(0);
        sleep(600);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);

        robot.runUsingEncoders(-265, 0.8, 3);
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);
        robot.runUsingEncoders(1600, 0.7, 4);

        robot.ridicareBratStanga.setPower(0.8);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);
        robot.ghearaStanga.setPosition(0.5);
        sleep(500);

        robot.runUsingEncoders(-150, 0.5, 2);

        robot.ridicareBratStanga.setPower(-0.85);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-300, 1, 4);

    }



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        //robot.zeroPowerBeh();
        waitForStart();

        telemetry.addData("Status: ", "ready");
        telemetry.update();

        if (opModeIsActive()) {

            robot.senzorStanga.enableLed(true);
            robot.senzorDreapta.enableLed(true);
            robot.ghearaStanga.setPosition(0.5);
            robot.ghearaDreapta.setPosition(0.5);
            robot.ridicareBratStanga.setPower(0.5);
            robot.scripeteStanga.setPower(0.4);
            robot.scripeteDreapta.setPower(0.4);
            robot.ridicareBratDreapta.setPower(0.5);

            robot.runUsingEncoders(robot.conversieCmToTick(620), 0.4, 4);

            robot.ridicareBratStanga.setPower(0);
            robot.scripeteStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);
            robot.scripeteDreapta.setPower(0);

            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
                robot.forward(0.25);
                telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
                telemetry.update();
            }

            // while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 75) {
            //     robot.forward(-0.15);
            //     telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            //     telemetry.update();
            // }
            robot.stopDriving();

            sleep(800);

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