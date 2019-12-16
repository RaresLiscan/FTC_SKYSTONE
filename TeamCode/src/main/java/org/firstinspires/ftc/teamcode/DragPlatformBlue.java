package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mutare fundatie - Albastru")
public class DragPlatformBlue extends LinearOpMode {

    private RobotMap robot = null;

    ElapsedTime runtime = new ElapsedTime();

    private void strafe45 (int ticks, double power, int timeout) {

        robot.dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.stangaSpate.setTargetPosition(ticks);
        robot.dreaptaFata.setTargetPosition(-ticks);

        robot.stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.dreaptaFata.setPower(power);
        robot.stangaSpate.setPower(power);

        runtime.reset();
        while (robot.dreaptaFata.isBusy() && robot.stangaSpate.isBusy() && runtime.seconds() < timeout && opModeIsActive());

        robot.stangaSpate.setPower(0);
        robot.dreaptaSpate.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap);
        robot.zeroPowerBeh();
        waitForStart();

        if (opModeIsActive()) {

            robot.strafe(-1250, 0.7, 3);

            robot.runUsingEncoders(3000, 0.4, 5);

            robot.ridicareBratStanga.setPower(0.8);
            robot.ridicareBratDreapta.setPower(0.8);
            sleep(1000);

            robot.runUsingEncoders(-3000, 0.4, 5);

            robot.ridicareBratDreapta.setPower(-0.8);
            robot.ridicareBratStanga.setPower(-0.8);
            sleep(1000);

            robot.strafe(5000, 0.6, 4);

        }

    }
}
