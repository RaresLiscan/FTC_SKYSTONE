package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Albastru 5p")
@Disabled
public class BlueAutonomous1 extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(350, 1, 2);

            robot.strafe(-700, 0.6, 3);

            robot.runUsingEncoders(-100, 1, 1);

            double correction = robot.maintainAngle();
            robot.rotateEncoders(robot.conversieDegreesToTicks(correction + 90), 1, 1);

        }

    }
}
