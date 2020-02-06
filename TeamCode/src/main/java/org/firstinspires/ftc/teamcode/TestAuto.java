package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test acceleratie")
@Disabled
public class TestAuto extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {
            robot.runUsingEncoders(2500, 1, 3);
        }
    }
}
