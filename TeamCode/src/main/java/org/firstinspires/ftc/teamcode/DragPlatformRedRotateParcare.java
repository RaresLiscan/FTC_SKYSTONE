package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Rosu rotire fundatie perete")
 @Disabled
public class DragPlatformRedRotateParcare extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        // robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(robot.conversieCmToTick(100), 0.4, 1);

            robot.strafe(845, 0.5, 3);//870

            double correction = robot.maintainAngle();
            telemetry.addData("Correction: ", correction);
            telemetry.update();
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction - 3), 1 , 1);

            robot.runUsingEncoders(robot.conversieCmToTick(600), 0.5, 3);//2600

            double conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
            robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.3, 3);

            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(700);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.rotateConstantSpeed(-15, 1, 5);//15

            robot.runUsingEncoders(-robot.conversieCmToTick(390), 1, 2);//1800

            robot.rotateConstantSpeed(-61, 1, 5);//70

            robot.runUsingEncoders(robot.conversieCmToTick(470), 0.5, 2);//500

            robot.ridicareBratDreapta.setPower(-0.85);
            robot.ridicareBratStanga.setPower(-0.85);

            robot.runUsingEncoders(-robot.conversieCmToTick(100), 1, 3);

            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.strafe(1300, 1, 3);

            correction = robot.maintainAngle();
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction), 1, 1);

            robot.runUsingEncoders(-robot.conversieCmToTick(850), 1, 3);//3000

        }

    }
}
