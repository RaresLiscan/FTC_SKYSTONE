package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Rosu completa perete")
@Disabled
public class AutonomousRedParcare extends LinearOpMode {

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

        robot.ridicareBratStanga.setPower(1);
        robot.ridicareBratDreapta.setPower(1);
        sleep(600);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.rotateConstantSpeed(-15, 0.6, 5);//15

        robot.runUsingEncoders(-robot.conversieCmToTick(440), 1, 2);//1800

        robot.rotateConstantSpeed(-61, 0.7, 5);//70

        robot.runUsingEncoders(robot.conversieCmToTick(520), 0.3, 2);//500

        robot.ridicareBratDreapta.setPower(-1);
        robot.ridicareBratStanga.setPower(-1);

        robot.runUsingEncoders(-robot.conversieCmToTick(150), 1, 1);

        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.strafe(1200, 0.6, 3);

        double correction = robot.maintainAngle();
        robot.rotateEncoders(robot.conversieDegreesToTicks(correction), 1 ,1);

        robot.runUsingEncoders(-robot.conversieCmToTick(750), 1, 3);//3000

    }


    void skystoneRight() {
        telemetry.addData("Position: ", "right");
        telemetry.update();

        robot.runUsingEncoders(150, 0.75, 3);

        robot.ghearaDreapta.setPosition(1);
        sleep(600);

        robot.ridicareBratStanga.setPower(-0.85);
        robot.ridicareBratDreapta.setPower(-0.85);
        sleep(700);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.runUsingEncoders(-200, 0.8, 3);
        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);

        robot.runUsingEncodersCorrection(robot.conversieCmToTick(2230), 1, 9);
        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
        }
        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 190), 0.3, 4);
        robot.ghearaDreapta.setPosition(0.5);
        rotateFundation();
    }

    void skystoneLeft() {
        telemetry.addData("Position: ", "left");
        telemetry.update();
        robot.runUsingEncoders(150,0.8, 3);

        robot.ghearaStanga.setPosition(0);
        sleep(300);
        robot.ridicareBratStanga.setPower(-1);
        robot.ridicareBratDreapta.setPower(-1);
        sleep(500);
        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.runUsingEncoders(-robot.conversieCmToTick(200), 1, 3);

        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 0.7, 2);
        robot.runUsingEncoders(robot.conversieCmToTick(2230), 1, 9);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.3);
        }

        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 0.7, 2);
        conversieCmTick =robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 190), 0.35, 5);
        robot.ghearaStanga.setPosition(0.5);
        rotateFundation();

    }

    void noSkystone() {
        telemetry.addData("Position: ", "no skystone");
        telemetry.update();

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) <= 75) {
            robot.forward(-0.25);
        }

        robot.strafe(-750, 0.6, 3);//-800, 1
        double correction = robot.maintainAngle();
        telemetry.addData("Corectia de unghi: ", correction);
        telemetry.update();
        robot.rotateEncoders(robot.conversieDegreesToTicks(correction - 3), 1, 1);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
            robot.forward(0.25);
        }


        robot.runUsingEncoders(150, 0.8, 3);
        robot.ghearaStanga.setPosition(0);
        sleep(500);

        robot.ridicareBratStanga.setPower(-0.85);
        robot.ridicareBratDreapta.setPower(-0.85);

        robot.runUsingEncoders(-200, 0.5, 3);

        robot.ridicareBratStanga.setPower(0);
        robot.ridicareBratDreapta.setPower(0);

        robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);

        robot.runUsingEncoders(robot.conversieCmToTick(2230), 1, 9);

        while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 350) {
            robot.forward(0.35);
        }

        robot.rotateEncoders(robot.conversieDegreesToTicks(90), 1, 2);
        conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
        robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 190), 0.3, 4);
        robot.ghearaStanga.setPosition(0.5);
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
            robot.ghearaDreapta.setPosition(0.5);
            robot.ridicareBratStanga.setPower(0.5);
            robot.ridicareBratDreapta.setPower(0.5);
            robot.scripeteStanga.setPower(0.4);
            robot.scripeteDreapta.setPower(0.4);

            robot.runUsingEncoders(robot.conversieCmToTick(620), 0.4, 4);

            robot.ridicareBratStanga.setPower(0);
            robot.scripeteStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);
            robot.scripeteDreapta.setPower(0);

            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
                robot.forward(0.25);
            }
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