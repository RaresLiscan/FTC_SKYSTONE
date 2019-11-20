package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="TeleOPTestAndroidStudio")
public class TestTeleOP extends LinearOpMode {

    RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double rotate = gamepad1.right_stick_x * 0.35;
            double strafe = gamepad1.left_stick_x * 0.65;
            double forward = -gamepad1.left_stick_y * 0.35;

            double dF, dS, sS, sF;


            if (Math.abs(forward) >= Math.abs(strafe)) {
                sS = forward + rotate;
                dF = -forward + rotate;
                sF = forward + rotate;
                dS = -forward + rotate;
            } else {
                sS = -strafe + rotate;
                dF = strafe + rotate;
                sF = strafe + rotate;
                dS = -strafe + rotate;
            }


            robot.stangaSpate.setPower(sS);
            robot.dreaptaFata.setPower(dF);
            robot.stangaFata.setPower(sF);
            robot.dreaptaSpate.setPower(dS);

            robot.scripeteStanga.setPower(gamepad2.left_stick_y * 0.6);

            if (gamepad2.a) robot.gheara.setPosition(0.5);
            if (gamepad2.b) robot.gheara.setPosition(1);

            robot.ridicareBratStanga.setPower(gamepad2.right_stick_y * 0.4);

        }
    }
}
