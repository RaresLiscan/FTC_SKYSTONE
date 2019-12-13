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

            robot.ghearaDreapta.setPosition(0.5);
            //Ridicarea bratului
            robot.ridicareBratDreapta.setPower(0.6);
            robot.scripeteDreapta.setPower(0.5);
            sleep(1000);

            //Deplasare catre cub
            robot.runUsingEncoders(6250, 0.8, 5);

            //Prinderea mineralului si ridicarea usoara a scripetelul
            robot.ghearaDreapta.setPosition(1);
            sleep(500);

            //Coborarea bratului
            robot.ridicareBratDreapta.setPower(-0.85);
            sleep(700);

            //Miscare inapoi ca sa poata trece pe sub skybridge
            robot.runUsingEncoders(-2750, 1, 5);

            //Miscare catre building zone
            robot.rotate(80, 0.8, 2);
            //12250 - pana la a doua gaura
            robot.runUsingEncoders(12250, 1, 9);

            //Indreptare catre platforma si miscare catre ea
            robot.rotate(-80, 0.8, 2);
            robot.runUsingEncoders(1750, 1, 3);

            //Ridicarea bratelor si plasarea mineralului pe platforma
//            robot.ridicareBratDreapta.setPower(0.8);
//            robot.scripeteDreapta.setPower(-0.9);
//            robot.ridicareBratStanga.setPower(-0.8);
//            robot.scripeteStanga.setPower(-0.9);
//            sleep(1000);
//            robot.scripeteDreapta.setPower(0.6);
//            sleep(700);
//            robot.ghearaDreapta.setPosition(0.5);
//            sleep(500);

            robot.macaraDreaptaEncoder(-180, 0.7, 2);

            robot.ridicareBratDreapta.setPower(0.8);
            robot.ridicareBratStanga.setPower(0.8);
            sleep(700);

            robot.ghearaDreapta.setPosition(0.5);

            robot.strafe(-2000, 1, 5);

            robot.macaraStDrEnc(95, 0.6, 2);

            robot.runUsingEncoders(-3600, 1, 4);
//            robot.rotate(90, 1, 2);
//
//            //Ridicarea bratului pentru a putea sa ne deplasam
//            robot.scripeteDreapta.setPower(-0.9);
//            robot.ridicareBratDreapta.setPower(-0.6);
//            robot.ridicareBratStanga.setPower(0.6);
//            robot.scripeteStanga.setPower(-0.9);
//            sleep(750);
//            robot.scripeteDreapta.setPower(0);
//            robot.ridicareBratDreapta.setPower(0);
//            robot.ridicareBratStanga.setPower(0);
//            robot.scripeteStanga.setPower(0);
//
//
//            //Miscare sub skybridge
//            robot.runUsingEncoders(-8000, 0.9, 2);


        }

    }
}
