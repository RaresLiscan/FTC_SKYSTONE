package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Mutare fundatie - Rosu")
//@Disabled
public class DragPlatformRed extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            robot.strafe(1250, 0.7, 3);

            robot.runUsingEncoders(2800, 0.4, 5);

            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(1000);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.runUsingEncoders(-3050, 0.4, 5);

            robot.ridicareBratDreapta.setPower(-0.8);
            robot.ridicareBratStanga.setPower(-0.8);
            sleep(1000);
            robot.ridicareBratStanga.setPower(0);
            robot.ridicareBratDreapta.setPower(0);

            robot.strafe(-5150, 0.6, 4);
        }

    }
}
