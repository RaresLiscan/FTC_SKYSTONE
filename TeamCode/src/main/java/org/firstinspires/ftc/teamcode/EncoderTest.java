package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Encoders")
@Disabled
public class EncoderTest extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {
            robot.strafeCorrectionTest(5000, 0.6, 4, telemetry);


        }

    }
}
