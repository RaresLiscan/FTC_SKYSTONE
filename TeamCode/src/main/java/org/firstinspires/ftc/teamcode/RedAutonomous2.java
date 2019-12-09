package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedAutonomous2")
@Disabled
public class RedAutonomous2 extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {
            robot.runUsingEncoders(robot.cmToTicks(5), 0.6, 4);
            robot.rotate(-90, 0.4, 3);
            robot.runUsingEncoders(robot.cmToTicks(40), 0.6, 5);
            robot.rotate(90, 0.4, 4);
            robot.ridicareBratStanga.setPower(0.3);
            robot.ridicareBratDreapta.setPower(-0.3);
            sleep(3000);
            robot.runUsingEncoders(-robot.cmToTicks(10), 0.4, 4);
            robot.ridicareBratDreapta.setPower(0.3);
            robot.ridicareBratStanga.setPower(-0.3);
            sleep(3000);
            robot.strafe(robot.cmToTicks(40), 0.5, 4);
        }
    }
}
