package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Albastru completa")

public class AutonomousBlue extends LinearOpMode {

    private RobotMap robot = null;
    double colorError = 18;
    double conversieCmTick = 0;


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

        robot.rotate(15, 0.6, 5);

        robot.runUsingEncoders(-1750, 0.7, 5);//1800

        robot.rotate(70, 0.4, 5);

        robot.ridicareBratDreapta.setPower(-0.8);
        robot.ridicareBratStanga.setPower(-0.8);
        sleep(1000);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.runUsingEncoders(1150, 0.7, 5);//500
        sleep(500);
        robot.runUsingEncoders(-3650, 0.9, 5);//3000

        robot.rotate(-78, 0.7, 5);

        double correction = robot.checkDirection();
        robot.rotate((int) correction, 0.2, 2);

        sleep(1000);
    }


    void skystoneLeft() {
        telemetry.addData("Position: ", "left");
        telemetry.update();
        robot.strafe(-1050, 0.65, 3);//1050
        double correction = robot.checkDirection();
        robot.rotate((int) correction, 0.2, 2);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 65) {
            robot.forward(-0.15);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.15);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        robot.runUsingEncoders(300, 0.8, 3);
        robot.ghearaDreapta.setPosition(1);
        sleep(600);
        robot.ridicareBratDreapta.setPower(-0.85);
        sleep(700);
        robot.ridicareBratDreapta.setPower(0);
        robot.runUsingEncoders(-600, 0.9, 3);
        robot.rotate(78, 0.7, 2);
        // robot.runUsingEncoders(7300, 1, 9);
        robot.runUsingEncoders(robot.conversieCmToTick(2000), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.rotate(-78, 0.7, 3);
        // while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 60) {
        //     robot.forward(0.3);
        //     telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
        //     telemetry.update();
        // }
        // while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 60) {
        //     robot.forward(-0.3);
        //     telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
        //     telemetry.update();
        // }

        //robot.runUsingEncoders(680, 0.4, 3);//550
        // robot.runUsingEncoders(400, 0.4, 3);//550
        // sleep(500);
        // conversieCmTick = robot.senzorDistanta.rawUltrasonic() * 10;
        // robot.runUsingEncoders((int)conversieCmTick, 0.4, 3);
        //sleep(10000);
        //conversieCmTick = 0;
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.4, 4);
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
        robot.runUsingEncoders(-800, 0.6, 3);
        robot.rotate(78, 0.7, 2);
        // robot.runUsingEncoders(7700, 1, 9);
        robot.runUsingEncoders(robot.conversieCmToTick(2230), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.2);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.rotate(-78, 0.7, 3);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.4, 4);
        //robot.runUsingEncoders(680, 0.5, 3);//550
        robot.ghearaDreapta.setPosition(0.5);
        robot.macaraDreaptaEncoder(-70, 0.7, 2);
        rotateFundation();
    }

    void noSkystone() {
        telemetry.addData("Position: ", "no skystone");
        telemetry.update();
        robot.strafe(950, 0.65, 3);//1000
        double correction = robot.checkDirection();
        robot.rotate((int) correction, 0.2, 2);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.15);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 65) {
            robot.forward(-0.15);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        robot.runUsingEncoders(300, 0.6, 3);
        robot.ghearaDreapta.setPosition(1);
        sleep(600);
        robot.ridicareBratDreapta.setPower(-0.85);
        sleep(700);
        robot.ridicareBratDreapta.setPower(0);
        robot.runUsingEncoders(-620, 0.8, 3);
        robot.rotate(78, 0.7, 2);
        robot.runUsingEncoders(7700, 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.rotate(-78, 0.7, 3);

        // robot.runUsingEncoders(680, 0.4, 3);//550
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.4, 4);

        robot.ghearaDreapta.setPosition(0.5);
        robot.macaraDreaptaEncoder(-70, 0.7, 2);
        rotateFundation();

    }




    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        telemetry.addData("Status: ", "ready");
        telemetry.update();

        if (opModeIsActive()) {

            robot.senzorStanga.enableLed(true);
            robot.senzorDreapta.enableLed(true);
            conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
            conversieCmTick -= 60;
            telemetry.addData("Distance: ", conversieCmTick);
            telemetry.update();
            robot.ghearaDreapta.setPosition(0.5);
            robot.ridicareBratDreapta.setPower(0.85);
            robot.scripeteDreapta.setPower(0.85);
            sleep(600);
            robot.ridicareBratDreapta.setPower(0);
            robot.scripeteDreapta.setPower(0);

            robot.runUsingEncoders(robot.conversieCmToTick(700), 0.9, 4);

            // robot.runUsingEncoders(2500, 0.6, 4);//2550


            // while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 65) {
            //     robot.forward(-0.15);
            //     // telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            //     // telemetry.update();
            // }

            // while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            //     robot.forward(0.15);
            //     // telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            //     // telemetry.update();
            // }

            // robot.stopDriving();


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