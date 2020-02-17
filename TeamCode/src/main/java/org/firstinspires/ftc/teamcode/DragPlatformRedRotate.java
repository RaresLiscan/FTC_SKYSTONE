package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Rosu rotire fundatie bridge")
@Disabled
public class DragPlatformRedRotate extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            robot.strafe(1700, 0.7, 3);//1700

            robot.runUsingEncoders(2500, 0.4, 5);//2600

            double conversieCmTick = robot.senzorDistantaRev.getDistance(DistanceUnit.MM);
            robot.runUsingEncoders(robot.conversieCmToTick(conversieCmTick), 0.4, 4);

            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(900);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.rotateConstantSpeed(-15, 0.7, 5);//15

            robot.runUsingEncoders(-1700, 0.7, 5);//1800

            robot.rotateConstantSpeed(-70, 0.7, 5);//70

            robot.runUsingEncoders(600, 0.6, 5);//500
            sleep(500);

            robot.ridicareBratDreapta.setPower(-0.8);
            robot.ridicareBratStanga.setPower(-0.8);
            sleep(900);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

//        robot.strafeCorrectionTest(500, 0.65, 3, telemetry);

            robot.runUsingEncoders(-450, 1, 2);

            robot.strafe(-700, 0.7, 2);

            robot.runUsingEncoders(-3200, 1, 5);//3000

            robot.ghearaDreapta.setPosition(0.87);
            sleep(1000);
        }

    }
}
