package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Albastru rotire fundatie bridge")
@Disabled
public class DragPlatformBlueRotate extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(robot.conversieCmToTick(100), 1, 1);

            robot.strafe(-870, 0.6, 2);//1700

            double correction = robot.maintainAngle();
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction + 4), 1, 1);

            robot.runUsingEncoders(robot.conversieCmToTick(700), 1, 3);//2600

            double conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
            robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.3, 3);

            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(700);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);
//
            robot.rotateConstantSpeed(15, 1, 1);

            robot.runUsingEncoders(-robot.conversieCmToTick(420), 1, 2);//1800
//
            robot.rotateConstantSpeed(63, 1, 3);
//
            robot.runUsingEncoders(robot.conversieCmToTick(270), 0.5, 2);//500
//
            robot.ridicareBratDreapta.setPower(-0.85);
            robot.ridicareBratStanga.setPower(-0.85);
//
            robot.runUsingEncoders(-robot.conversieCmToTick(100), 1, 3);

            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

//
            robot.strafe(200, 1, 1);
//
            robot.runUsingEncoders(-robot.conversieCmToTick(900), 1, 3);//3000
//
        }

    }
}