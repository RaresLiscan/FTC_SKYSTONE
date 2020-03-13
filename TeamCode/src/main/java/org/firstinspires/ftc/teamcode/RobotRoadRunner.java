package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RoadRunner")
@Disabled
public class RobotRoadRunner extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor stangaFata = null;
    private DcMotor dreaptaFata = null;
    private DcMotor stangaSpate = null;
    private DcMotor dreaptaSpate = null;

    private void runUsingEncoders(int distance, double power, double timeout) {
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaFata.setTargetPosition(distance);
        stangaSpate.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(-distance);
        dreaptaSpate.setTargetPosition(-distance);

        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stangaFata.setPower(power);
        dreaptaFata.setPower(power);
        dreaptaSpate.setPower(power);
        stangaSpate.setPower(power);

        runtime.reset();

        while (stangaFata.isBusy() && stangaSpate.isBusy() && dreaptaFata.isBusy() && dreaptaSpate.isBusy() && runtime.seconds() < timeout);

        stopDriving();
    }

    void stopDriving() {
        stangaSpate.setPower(0);
        stangaFata.setPower(0);
        dreaptaSpate.setPower(0);
        dreaptaFata.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");

        waitForStart();

        if (opModeIsActive()) {
            runUsingEncoders(2000, 0.6, 4);
        }

    }
}
