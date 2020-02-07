package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Albastru rotire fundatie perete")
//@Disabled
public class DragPlatformBlueRotateParcare extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            robot.strafe(-1800, 0.7, 3);//1700

            robot.runUsingEncoders(3000, 0.4, 5);//2600

            double conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
            robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.4, 4);

            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(1000);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.rotate(15, 0.4, 5);

            robot.runUsingEncoders(-1750, 0.4, 5);//1800

            robot.rotate(70, 0.4, 5);

            robot.ridicareBratDreapta.setPower(-0.8);
            robot.ridicareBratStanga.setPower(-0.8);
            sleep(1000);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.runUsingEncoders(700, 0.4, 5);//500

            robot.runUsingEncoders(-300, 0.7, 5);

            robot.strafe(-1600, 0.7, 3);

            robot.runUsingEncoders(-3350, 0.7, 5);//3000

            robot.strafe(-600, 0.7, 2);

            robot.rotate(-83, 0.4, 5);

            robot.ghearaStanga.setPosition(0.23);
            sleep(1000);
        }

    }
}