package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedAutonomous Test")
public class RedAutonomousTest extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            //Ridicarea bratului
            robot.ridicareBratStanga.setPower(0.3);
            sleep(1500);

            robot.scripeteStanga.setPower(0.5);
            sleep(1000);

            //Deplasare catre cub
            robot.runUsingEncoders(7000, 0.8, 5);

            //Prinderea mineralului si ridicarea usoara a scripetelul
            robot.ghearaStanga.setPosition(1);
            sleep(500);

            //Coborarea bratului
            robot.ridicareBratStanga.setPower(-0.4);
            sleep(1000);

            //Miscare inapoi ca sa poata trece pe sub skybridge
            robot.runUsingEncoders(-3500, 0.9, 5);

            //Miscare catre building zone
            robot.rotate(-80, 0.8, 2);
            robot.runUsingEncoders(12250, 1, 9);

            //Indreptare catre platforma si miscare catre ea
            robot.rotate(80, 0.8, 2);
            robot.runUsingEncoders(1350, 0.7, 2);

            //Ridicarea bratelor si plasarea mineralului pe platforma
            robot.ridicareBratDreapta.setPower(0.8);
            robot.scripeteDreapta.setPower(-0.9);
            robot.ridicareBratStanga.setPower(-0.8);
            robot.scripeteStanga.setPower(-0.9);
            sleep(1000);
            robot.scripeteDreapta.setPower(0.6);
            sleep(700);
            robot.ghearaStanga.setPosition(0.5);
            sleep(500);

            robot.runUsingEncoders(-2300, 1, 4);
            robot.rotate(-90, 1, 2);

            //Ridicarea bratului pentru a putea sa ne deplasam
            robot.scripeteDreapta.setPower(-0.9);
            robot.ridicareBratDreapta.setPower(-0.6);
            robot.ridicareBratStanga.setPower(0.6);
            robot.scripeteStanga.setPower(-0.9);
            sleep(750);
            robot.scripeteDreapta.setPower(0);
            robot.ridicareBratDreapta.setPower(0);
            robot.ridicareBratStanga.setPower(0);
            robot.scripeteStanga.setPower(0);

            //Miscare sub skybridge
            robot.runUsingEncoders(-8000, 0.9, 2);
        }
    }
}
