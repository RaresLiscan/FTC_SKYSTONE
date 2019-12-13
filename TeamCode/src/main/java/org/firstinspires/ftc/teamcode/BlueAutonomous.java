package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueAutonomous")
//@Disabled
public class BlueAutonomous extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {

            robot.macaraStangaEncoder(-144, 0.6, 5);

        }

    }
}
