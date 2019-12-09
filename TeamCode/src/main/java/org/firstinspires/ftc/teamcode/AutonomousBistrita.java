package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomie Albastru")
public class AutonomousBistrita extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            //Pozitionare catre cub
            robot.strafe(1400, 0.7, 3);
////
////            //Ridicarea bratului
            robot.ridicareBratDreapta.setPower(0.3);
            sleep(1500);
//
            robot.scripeteDreapta.setPower(0.5);
            sleep(1000);
//
//            //Deplasare catre cub
            robot.runUsingEncoders(7000, 0.8, 5);
//
//            //Prinderea mineralului si ridicarea usoara a scripetelul
            robot.ghearaDreapta.setPosition(1);
            sleep(500);
//
//            //Coborarea bratului
            robot.ridicareBratDreapta.setPower(-0.4);
            sleep(1000);
//
//            //Miscare inapoi ca sa poata trece pe sub skybridge
            robot.runUsingEncoders(-3500, 0.9, 5);
//
//            //Miscare catre building zone
            robot.rotate(80, 0.8, 2);
            robot.runUsingEncoders(12250, 1, 9);

            //Indreptare catre platforma si miscare catre ea
            robot.rotate(-80, 0.8, 2);
            robot.runUsingEncoders(1350, 0.7, 2);

            //Ridicarea bratului si plasarea mineralului pe platforma
            robot.ridicareBratDreapta.setPower(0.4);
            robot.scripeteDreapta.setPower(-0.4);
            sleep(1000);
            robot.scripeteDreapta.setPower(0.6);
            sleep(500);
            robot.ghearaDreapta.setPosition(0.5);
            sleep(500);

            //Ridicarea bratului pentru a putea sa ne deplasam
            robot.scripeteDreapta.setPower(-0.9);
            robot.ridicareBratDreapta.setPower(-0.5);
            sleep(600);
            robot.scripeteDreapta.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            //Indepartare dinspre platforma
            robot.runUsingEncoders(-1200, 0.9, 2);

            //Rotire catre skybridge si miscare sub el
            robot.rotate(-80, 0.8, 2);
            robot.runUsingEncoders(8000, 1, 6);

//
//            //Miscare in fata catre platforma
//            robot.runUsingEncoders(7000, 0.6, 4);
//
//            //Ridicarea bratului si lasarea scripetelui
//            robot.ridicareBratDreapta.setPower(-0.3);
//            sleep(2000);
//            robot.scripeteStanga.setPower(0.5);
//            sleep(1500);
//
//            //Eliberarea stone-ului
//            robot.ghearaDreapta.setPosition(0.5);
//
//            //Miscare inapoi si plasare sub skybridge
//            robot.runUsingEncoders(-6500, 0.6, 4);
//            robot.strafe(5000, 0.6, 4);

        }

    }
}
