package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Rosu completa perete")
//@Disabled
public class AutonomousRedParcare extends LinearOpMode {

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
        sleep(900);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.rotateConstantSpeed(-15, 0.7, 5);//15

        robot.runUsingEncoders(-1700, 0.7, 5);//1800

        robot.rotateConstantSpeed(-70, 0.7, 5);//70

        robot.ridicareBratDreapta.setPower(-0.8);
        robot.ridicareBratStanga.setPower(-0.8);
        sleep(900);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.runUsingEncoders(1300, 0.6, 5);//500
        sleep(500);

//        robot.strafeCorrectionTest(500, 0.65, 3, telemetry);

        robot.runUsingEncodersCorrection(-300, 1, 2);

        robot.strafe(1750, 0.7, 2);

        robot.runUsingEncoders(-3350, 1, 5);//3000

//        robot.rotate(78, 0.7, 5);//-78
//
//        double correction = robot.checkDirection();
//        robot.rotate((int) correction, 0.2, 2);

        robot.ghearaDreapta.setPosition(0.87);
        sleep(1000);
    }


    void skystoneRight() {
        telemetry.addData("Position: ", "right");
        telemetry.update();
        //robot.runUsingEncoders(-100, 0.4,3);
        robot.strafe(950, 0.65, 3);//1000
        double correction = robot.checkDirection();
        robot.rotate((int) correction, 0.2, 2);

//        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 65) {
//            robot.forward(-0.15);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
//        }

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.15);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }

        robot.runUsingEncoders(400, 0.75, 3);
        robot.ghearaStanga.setPosition(0);
        sleep(600);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(700);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-800, 0.8, 3);
        robot.rotate(-87, 1, 2);//-79
        //robot.runUsingEncoders(7300, 1, 9);
        robot.runUsingEncodersCorrection(7000, 1, 9);//7100
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 370) {
            robot.forward(0.3);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.rotate(89, 1, 3);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.4, 4);

        //robot.runUsingEncoders(800, 0.4, 3);//550
        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-110, 0.7, 2);
        rotateFundation();
    }

    void skystoneLeft() {
//        telemetry.addData("Position: ", "left");
//        telemetry.update();
        robot.runUsingEncoders(300,0.6, 3);
        robot.ghearaStanga.setPosition(0);
        sleep(600);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(700);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-700, 0.75, 3);
        robot.rotate(-87, 1, 2);//
        robot.runUsingEncodersCorrection(7300, 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.4);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }
        robot.rotate(89, 1, 3);//
        conversieCmTick =robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.4, 5);

        //robot.runUsingEncoders(800, 0.5, 3);//550
        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-110, 0.7, 2);
        rotateFundation();
    }

    void noSkystone() {
        telemetry.addData("Position: ", "no skystone");
        telemetry.update();
        robot.strafe(-910, 0.65, 3);//1000
        double correction = robot.checkDirection();
        robot.rotate((int) correction, 0.2, 2);

//        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
//            robot.forward(0.15);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
//        }

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 65) {
            robot.forward(-0.15);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }

        robot.runUsingEncoders(350, 0.8, 3);//
        robot.ghearaStanga.setPosition(0);
        sleep(600);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(700);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-800, 0.8, 3);
        robot.rotate(-87, 1, 2);
        robot.runUsingEncodersCorrection(7900, 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.4);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.rotate(89, 1, 3);

        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.5, 4);

        //robot.runUsingEncoders(800, 0.4, 3);//550

        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-110, 0.7, 2);
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
//            conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
//            conversieCmTick -= 60;
//            telemetry.addData("Distance: ", conversieCmTick);
//            telemetry.update();
            robot.ghearaStanga.setPosition(0.5);
            robot.ridicareBratStanga.setPower(0.5);
            robot.scripeteStanga.setPower(0.5);
            sleep(1000);
            robot.ridicareBratStanga.setPower(0);
            robot.scripeteStanga.setPower(0);
            //robot.runUsingEncoders(2400, 0.6, 4);//2550
//            sleep(1000);

            robot.runUsingEncodersCorrection(robot.conversieCmToTick(700), 0.9, 4);

            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
                robot.forward(0.15);
//                 telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//                 telemetry.update();
            }

            // while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 75) {
            //     robot.forward(-0.15);
            //     telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            //     telemetry.update();
            // }
            robot.stopDriving();

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