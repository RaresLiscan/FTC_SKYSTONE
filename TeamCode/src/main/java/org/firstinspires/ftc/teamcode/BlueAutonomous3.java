package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomie Albastru")
@Disabled
public class BlueAutonomous3 extends LinearOpMode {

    RobotMap robot = null;

    //Ridicare macara pe drum
    //Coborare brat pe drum

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.ghearaDreapta.setPosition(0.5);
            //Ridicarea bratului
            robot.ridicareBratDreapta.setPower(0.8);
            robot.scripeteDreapta.setPower(0.8);
            sleep(800);
            robot.ridicareBratDreapta.setPower(0);
            robot.scripeteDreapta.setPower(0);

            //Deplasare catre cub
            robot.runUsingEncoders(3000, 0.6, 5);

            //Prinderea mineralului si ridicarea usoara a scripetelul
            robot.ghearaDreapta.setPosition(1);
            sleep(500);

            //Coborarea bratului
            robot.ridicareBratDreapta.setPower(-0.85);
            sleep(700);
            robot.ridicareBratDreapta.setPower(0);

            //Miscare inapoi ca sa poata trece pe sub skybridge
            robot.runUsingEncoders(-1000, 0.6, 5);

            //Miscare catre building zone
            robot.rotate(78, 0.7, 2);
            robot.runUsingEncoders(6700, 0.6, 9);

//          Indreptare catre platforma si miscare catre ea
            robot.rotate(-80, 0.8, 2);
            robot.runUsingEncoders(1050, 0.6, 3);
            robot.ghearaDreapta.setPosition(0.5);
            sleep(600);
            robot.macaraDreaptaEncoder(-70, 0.7, 2);


            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(800);

            robot.runUsingEncoders(-3150, 0.4, 5);

            robot.ridicareBratDreapta.setPower(-0.8);
            robot.ridicareBratStanga.setPower(-0.8);
            sleep(800);

            robot.strafe(5500, 0.6, 4);

        }

    }
}
