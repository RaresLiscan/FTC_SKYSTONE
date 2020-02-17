package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="TestRobotMap")
public class TestRobotMap extends LinearOpMode {

    RobotMapNoCrashTest robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMapNoCrashTest(hardwareMap, this);

        waitForStart();
        if (opModeIsActive()) {
            robot.runUsingEncoders(5000, 0.5, 3);
        }
    }
}
