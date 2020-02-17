package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomie Rosu")
@Disabled
public class RedAutonomous3 extends LinearOpMode {

    RobotMap robot = null;

    //Ridicare macara pe drum
    //Coborare brat pe drum

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.ghearaStanga.setPosition(0.5);
            //Ridicarea bratului
            robot.ridicareBratStanga.setPower(0.8);
            robot.scripeteStanga.setPower(0.8);
            sleep(800);
            robot.ridicareBratStanga.setPower(0);
            robot.scripeteStanga.setPower(0);

            //Deplasare catre cub
            robot.runUsingEncoders(3000, 0.6, 5);


            //Prinderea mineralului si ridicarea usoara a scripetelul
            robot.ghearaStanga.setPosition(0);
            sleep(500);

            //Coborarea bratului
            robot.ridicareBratStanga.setPower(-0.85);
            sleep(700);
            robot.ridicareBratStanga.setPower(0);

            //Miscare inapoi ca sa poata trece pe sub skybridge
            robot.runUsingEncoders(-1000, 0.6, 5);

            //Miscare catre building zone
            robot.rotate(-78, 0.7, 2);
            //12250 - pana la a doua gaura
            robot.runUsingEncoders(6700, 0.6, 9);

//            Indreptare catre platforma si miscare catre ea
            robot.rotate(80, 0.7, 2);
            robot.runUsingEncoders(1050, 0.6, 3);
            robot.ghearaStanga.setPosition(0.5);
            sleep(600);
            robot.macaraStangaEncoder(-70, 0.7, 2);


            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(800);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.runUsingEncoders(-3050, 0.4, 5);

            robot.ridicareBratDreapta.setPower(-0.8);
            robot.ridicareBratStanga.setPower(-0.8);
            sleep(800);
            robot.ridicareBratDreapta.setPower(0);
            robot.ridicareBratStanga.setPower(0);

            robot.strafe(-6000, 0.6, 9);

        }

    }
}
