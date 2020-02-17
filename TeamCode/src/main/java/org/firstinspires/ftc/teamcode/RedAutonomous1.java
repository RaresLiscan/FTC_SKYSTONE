package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomie Rosu 5p")
@Disabled
public class RedAutonomous1 extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(350, 0.6, 2);

            robot.strafe(1700, 0.6, 3);

            robot.rotate(85, 1, 2);

            robot.strafe(-450, 0.7, 2);

            robot.ghearaDreapta.setPosition(0.87);
            sleep(1000);

        }

    }
}
