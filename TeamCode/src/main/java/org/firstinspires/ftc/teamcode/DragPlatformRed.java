package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Rosu mutare fundatie")
@Disabled
public class DragPlatformRed extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(robot.conversieCmToTick(100), 1, 1);

            robot.strafe(850, 0.6, 2);//1700

            double correction = robot.maintainAngle();
            telemetry.addData("Correction: ", correction);
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction - 4), 1, 1);

            robot.runUsingEncoders(robot.conversieCmToTick(600), 0.5, 3);//2600

            double conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
            robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.3, 3);

            robot.ridicareBratStanga.setPower(1);
            robot.ridicareBratDreapta.setPower(1);
            sleep(600);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.rotateConstantSpeed(-15, 0.6, 5);//15

            robot.runUsingEncoders(-robot.conversieCmToTick(420), 1, 2);//1800

            robot.rotateConstantSpeed(-63, 0.7, 5);//70

            robot.runUsingEncoders(robot.conversieCmToTick(520), 0.3, 2);//500

            robot.ridicareBratDreapta.setPower(-1);
            robot.ridicareBratStanga.setPower(-1);

            robot.runUsingEncoders(-robot.conversieCmToTick(100), 1, 1);

            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.strafe(200, 1, 1);

            robot.runUsingEncoders(-robot.conversieCmToTick(800), 1, 3);//3000
        }

    }
}
