package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name = "Rosu test 2 stone")
@Disabled
public class AutoRed2stones extends LinearOpMode {

    private RobotMap robot = null;
    double colorError = 18;
    double conversieCmTick = 0;

    boolean checkSensor(double a, double b, double c) {
        if (Math.abs(a - b) > colorError || Math.abs(a - c) > colorError || Math.abs(b - c) > colorError) return false;
        return true;
    }


    void rotateFundation() {
        robot.ridicareBratStanga.setPower(0.9);
        robot.ridicareBratDreapta.setPower(0.9);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.rotateConstantSpeed(-15, 0.8, 5);//15

        robot.runUsingEncoders(-robot.conversieCmToTick(440), 1, 2);//1800

        robot.rotateConstantSpeed(-63, 0.8, 5);//70

        robot.runUsingEncoders(robot.conversieCmToTick(290), 0.6, 2);//500

        robot.ridicareBratDreapta.setPower(-0.85);
        robot.ridicareBratStanga.setPower(-0.85);

        robot.runUsingEncoders(-robot.conversieCmToTick(100), 1, 3);

        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);


        robot.strafe(-400, 1, 2);

        //robot.runUsingEncoders(-robot.conversieCmToTick(900), 1, 3);//3000

        robot.ghearaDreapta.setPosition(0.87);
        //sleep(1000);
    }


    void skystoneRight() {
        telemetry.addData("Position: ", "right");
        telemetry.update();

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.4);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.stopDriving();

        robot.strafe(660, 0.60, 3);//1000
        double correction = robot.checkDirection();
        robot.rotateConstantSpeed((int) correction, 0.35, 2);



        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.25);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }

        robot.stopDriving();

        robot.runUsingEncoders(200, 0.75, 3);
        robot.ghearaStanga.setPosition(0);
        sleep(600);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(700);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-400, 0.8, 3);
//        robot.rotate(-83, 1, 2);//-79
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);
        //robot.runUsingEncoders(7300, 1, 9);
        robot.runUsingEncodersCorrection(robot.conversieCmToTick(2000), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }
//        robot.rotate(84, 1, 3);
        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 90), 0.3, 4);
        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-95, 0.7, 2);
        rotateFundation();
    }

    void skystoneLeft() {
        telemetry.addData("Position: ", "left");
        telemetry.update();
        robot.runUsingEncoders(robot.conversieCmToTick(60),0.8, 3);
        robot.ghearaStanga.setPosition(0);
        sleep(250);
        robot.ridicareBratStanga.setPower(-1);
        sleep(400);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-robot.conversieCmToTick(200), 1, 3);
//        robot.rotate(-82, 1, 2);//
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);
        robot.runUsingEncodersCorrection(robot.conversieCmToTick(2230), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.4);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }
//        robot.rotate(85, 1, 3);//
        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        conversieCmTick =robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 90), 0.4, 5);
        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-95, 1, 2);
        rotateFundation();
    }

    void noSkystone() {
        telemetry.addData("Position: ", "no skystone");
        telemetry.update();

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 75) {
            robot.forward(-0.25);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }

        robot.strafe(-550, 0.65, 3);//1000
        double correction = robot.checkDirection();
        robot.rotateConstantSpeed((int) correction, 0.35, 2);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.25);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }


        robot.runUsingEncoders(150, 0.8, 3);//
        robot.ghearaStanga.setPosition(0);
        sleep(300);
        robot.ridicareBratStanga.setPower(-0.85);
        sleep(400);
        robot.ridicareBratStanga.setPower(0);
        robot.runUsingEncoders(-400, 0.8, 3);
//        robot.rotate(-83, 1, 2);
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);
        robot.runUsingEncodersCorrection(robot.conversieCmToTick(2230), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.35);
            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
//        robot.rotate(84, 1, 3);

        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 85), 0.3, 4);

        //robot.runUsingEncoders(800, 0.4, 3);//550

        robot.ghearaStanga.setPosition(0.5);
        robot.macaraStangaEncoder(-95, 0.8, 2);
        rotateFundation();

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
            robot.ridicareBratStanga.setPower(0.5);
            robot.scripeteStanga.setPower(0.4);

            robot.runUsingEncodersCorrection(robot.conversieCmToTick(620), 1, 4);

            robot.ridicareBratStanga.setPower(0);
            robot.scripeteStanga.setPower(0);

            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
                robot.forward(0.25);
//                 telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//                 telemetry.update();
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

            robot.runUsingEncoders(-robot.conversieCmToTick(1700), 1, 3);
            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
                robot.forward(0.4);
//            telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
//            telemetry.update();
            }
            robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);


        }

    }
}