package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotMap {

    public DcMotor stangaFata = null;
    public DcMotor stangaSpate = null;
    public DcMotor dreaptaFata = null;
    public DcMotor dreaptaSpate = null;
    public DcMotor ridicareBrat = null;
    public DcMotor scripeteSlide = null;
    public Servo gheara = null;

    public RobotMap (HardwareMap hardwareMap) {
        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");
//        ridicareBrat = hardwareMap.get(DcMotor.class, "ridicareBrat");
//        scripeteSlide = hardwareMap.get(DcMotor.class, "scripeteSlide");
//        gheara = hardwareMap.get(Servo.class, "gheara");
    }

    public void runUsingEncoders (int distance, double power, double timeout) {
        ElapsedTime runtime = new ElapsedTime();

        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaFata.setTargetPosition(distance);
        stangaSpate.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(distance);
        dreaptaSpate.setTargetPosition(distance);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stangaFata.setPower(power);
        stangaSpate.setPower(power);
        dreaptaFata.setPower(power);
        dreaptaSpate.setPower(power);

        runtime.reset();

        while (stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout);

        stopDriving();

    }

    public void stopDriving() {
        stangaFata.setPower(0);
        stangaSpate.setPower(0);
        dreaptaFata.setPower(0);
        dreaptaSpate.setPower(0);
    }

}
