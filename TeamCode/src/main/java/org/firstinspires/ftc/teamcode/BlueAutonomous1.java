package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Albastru 5p")
//@Disabled
public class BlueAutonomous1 extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {

            robot.runUsingEncoders(350, 1, 2);

            robot.strafe(-1700, 0.6, 3);

            robot.rotate(-85, 1, 2);

            robot.strafe(450, 0.7, 2);

            robot.ghearaStanga.setPosition(0.23);
            sleep(1000);
        }

    }
}
