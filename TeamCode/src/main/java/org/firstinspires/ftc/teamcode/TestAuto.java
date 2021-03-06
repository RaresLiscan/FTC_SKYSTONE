package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test rotire encoders")
@Disabled
public class TestAuto extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        waitForStart();

        if (opModeIsActive()) {

            robot.resetAngle();
            telemetry.addData("Initial angle: ", robot.getAngle());
            telemetry.update();

            robot.rotateEncoders(1200, 1, 3);

            sleep(1000);

            telemetry.addData("Angle: ", robot.getAngle());
            telemetry.update();

            sleep(10000);
        }
    }
}
