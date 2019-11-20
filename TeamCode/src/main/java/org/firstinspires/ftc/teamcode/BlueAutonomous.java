package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueAutonomous")
public class BlueAutonomous extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {

            robot.raiseServo(1000, 0.4, 10);
            robot.strafe(2000, 0.65, 4);
            robot.runUsingEncoders(2000, 0.5, 4);
            robot.gheara.setPosition(1);
            robot.runUsingEncoders(-500, 0.5, 4);
            robot.strafe(8000, 0.65, 10);
            robot.runUsingEncoders(500, 0.3, 3);

        }

    }
}
