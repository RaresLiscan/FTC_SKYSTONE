package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name = "Albastru SKYSTONE 3-6")
@Disabled
public class AlbastruSkystone36 extends LinearOpMode {

    private RobotMap robot = null;
    double conversieCmTick = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        double correction = robot.maintainAngle();

        // robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            //Primul skystone
            robot.ridicareBratDreapta.setPower(0.5);

            robot.ghearaDreapta.setPosition(0.5);

            robot.scripeteDreapta.setPower(0.3);


            robot.rotateEncoders(-robot.conversieDegreesToTicks(14), 0.5, 2);

            robot.runUsingEncoders(robot.conversieCmToTick(1100), 0.3, 4);

            robot.ghearaDreapta.setPosition(1);

            sleep(600);


            robot.macaraDreaptaEncoder(-70, 0.8, 2);

            //sleep(500);

            robot.runUsingEncoders(-robot.conversieCmToTick(300), 0.4, 4);

            robot.ridicareBratDreapta.setPower(-0.85);

            sleep(500);

            robot.rotateEncoders(robot.conversieDegreesToTicks(101), 0.8, 2);


            //Mers la perete
            robot.runUsingEncoders(robot.conversieCmToTick(2450), 1, 9);
            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 380) {
                robot.forward(0.4);
            }
            robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 1, 2);

            conversieCmTick = robot.senzorFataDreapta.getDistance(DistanceUnit.MM);
            robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick + 180), 0.3, 4);

            robot.ghearaDreapta.setPosition(0.5);

            sleep(500);

            //robot.macaraStangaEncoder(-95, 0.8, 2);
            //robot.macaraDreaptaEncoder(-70, 0.8, 2);

            robot.ridicareBratDreapta.setPower(0.5);
            robot.ridicareBratStanga.setPower(0.5);

            sleep(1000);


            //Rotirea fundatiei
            robot.rotateConstantSpeed(15, 0.7, 1);

            robot.runUsingEncoders(-robot.conversieCmToTick(440), 0.7, 2);//1800

            robot.rotateConstantSpeed(65, 0.7, 3);

            robot.runUsingEncoders(robot.conversieCmToTick(320), 0.25, 2);//500

            robot.ridicareBratDreapta.setPower(-0.85);
            robot.ridicareBratStanga.setPower(-0.85);


            //Mers spre al 2-lea skystone
            robot.runUsingEncoders(-robot.conversieCmToTick(1785), 1, 2);

            sleep(500);

            //double correction = robot.maintainAngle();
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction), 1, 2);//correction + 3

            robot.ghearaDreapta.setPosition(0.5);

            robot.rotateEncoders(-robot.conversieDegreesToTicks(90), 0.8, 2);

            robot.ridicareBratDreapta.setPower(0.5);
            sleep(700);

            robot.scripeteDreapta.setPower(0.3);


            while (robot.senzorDistanta.getDistance(DistanceUnit.MM) >= 65) {
                robot.forward(0.30);
                // telemetry.addData("mm", "%.2f mm", robot.senzorDistanta.getDistance(DistanceUnit.MM));
                // telemetry.update();
            }

            robot.runUsingEncoders(robot.conversieCmToTick(100), 0.7, 2);
            robot.ghearaDreapta.setPosition(1);
            sleep(500);
            robot.macaraDreaptaEncoder(-70, 0.8, 2);
            robot.ridicareBratDreapta.setPower(-0.85);
            sleep(500);
            robot.runUsingEncoders(-robot.conversieCmToTick(200), 0.7, 2);
            robot.rotateEncoders(robot.conversieDegreesToTicks(92), 0.8, 2);

            //robot.rotateEncoders(robot.conversieDegreesToTicks(correction + 3), 1, 2);

            robot.runUsingEncoders(robot.conversieCmToTick(1750), 1, 2);
            robot.ghearaDreapta.setPosition(0.5);
            sleep(500);
            robot.runUsingEncoders(-robot.conversieCmToTick(1100), 0.8, 2);
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction), 1, 2);//correction + 3
            sleep(500);



        }
    }
}
