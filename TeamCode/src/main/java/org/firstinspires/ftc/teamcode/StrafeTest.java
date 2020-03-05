package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Bag in el strafe")
@Disabled
public class StrafeTest extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {
            robot.strafe(-800, 0.5, 30);

            sleep(1000);
        }
    }
}
