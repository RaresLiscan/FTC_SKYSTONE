package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mutare fundatie - Albastru")
public class DragPlatform extends LinearOpMode {

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

            robot.scripeteDreapta.setPower(0.8);
            robot.scripeteStanga.setPower(0.8);
            sleep(800);

            robot.macaraStDrEnc(-80, 0.7, 4);

            robot.strafe(-2000, 1, 5);

            robot.runUsingEncoders(6500, 1, 7);

            robot.ridicareBratStanga.setPower(0.9);
            robot.ridicareBratDreapta.setPower(0.9);
            sleep(700);

            robot.runUsingEncoders(-700, 1, 5);

            strafe45(-5500, 1, 4);

            robot.rotate(85, 1, 5);

            robot.ridicareBratDreapta.setPower(-0.8);
            robot.ridicareBratStanga.setPower(-0.8);
            sleep(700);

            robot.runUsingEncoders(-5500, 1, 9);
        }

    }
}
