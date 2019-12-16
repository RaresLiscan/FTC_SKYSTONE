package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomie Albastru 5p")
public class BlueAutonomous1 extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(350, 1, 2);

            robot.strafe(-1700, 0.6, 3);
        }

    }
}
