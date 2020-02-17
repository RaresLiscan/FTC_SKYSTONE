package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotMapNoCrashTest {

    public DcMotor stangaFata;
    public DcMotor stangaSpate;
    public DcMotor dreaptaFata;
    public DcMotor dreaptaSpate;
    private LinearOpMode opMode;

    public RobotMapNoCrashTest(HardwareMap hardwareMap, LinearOpMode opMode) {
        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");

        this.opMode = opMode;
    }

    public void runUsingEncoders (int distance, double power, double timeout) {
        ElapsedTime runtime = new ElapsedTime();

        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaFata.setTargetPosition(-distance);
        stangaSpate.setTargetPosition(-distance);
        dreaptaFata.setTargetPosition(distance);
        dreaptaSpate.setTargetPosition(distance);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = 0.05;

        stangaFata.setPower(p);
        stangaSpate.setPower(p);
        dreaptaFata.setPower(p);
        dreaptaSpate.setPower(p);

        runtime.reset();

        while (opMode.opModeIsActive() &&  stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout) {
            if (p < power) p += 0.05;
            stangaFata.setPower(p);
            stangaSpate.setPower(p);
            dreaptaFata.setPower(p);
            dreaptaSpate.setPower(p);
        }
        stopDriving();
    }

    public void stopDriving() {
        stangaFata.setPower(0);
        stangaSpate.setPower(0);
        dreaptaFata.setPower(0);
        dreaptaSpate.setPower(0);
    }

}
