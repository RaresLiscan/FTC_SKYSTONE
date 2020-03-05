package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Rosu 5p")
@Disabled
public class RedAutonomous1 extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(350, 1, 2);

            robot.strafe(700, 0.6, 3);

            robot.runUsingEncoders(-200, 1, 1);

            double correction = robot.maintainAngle();
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction - 90), 1, 1);

        }

    }
}
