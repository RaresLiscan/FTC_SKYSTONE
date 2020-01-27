package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotMap {

    public DcMotor stangaFata = null;
    public DcMotor stangaSpate = null;
    public DcMotor dreaptaFata = null;
    public DcMotor dreaptaSpate = null;
    public DcMotor ridicareBratStanga = null;
    public DcMotor ridicareBratDreapta = null;
    public DcMotor scripeteStanga = null;
    public DcMotor scripeteDreapta = null;
    public Servo ghearaDreapta = null;
    public Servo ghearaStanga = null;
    public BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction, prevAngle;
    double coborareScripete = 0.5;
    public ColorSensor senzorStanga;
    public ColorSensor senzorDreapta;
    public ModernRoboticsI2cRangeSensor senzorDistanta;
    public Rev2mDistanceSensor senzorDistantaRev;


    public RobotMap (HardwareMap hardwareMap) {
        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");
        scripeteStanga = hardwareMap.get(DcMotor.class, "scripeteStanga");
        scripeteDreapta = hardwareMap.get(DcMotor.class, "scripeteDreapta");
        ridicareBratStanga = hardwareMap.get(DcMotor.class, "ridicareBratStanga");
        ridicareBratDreapta = hardwareMap.get(DcMotor.class, "ridicareBratDreapta");
        ghearaDreapta = hardwareMap.get(Servo.class, "ghearaDreapta");
        ghearaDreapta.setPosition(0.9);
        ghearaStanga = hardwareMap.get(Servo.class, "ghearaStanga");
        ghearaStanga.setPosition(0.155);
        senzorDreapta = hardwareMap.get(ColorSensor.class, "senzorDr");
        senzorStanga = hardwareMap.get(ColorSensor.class, "senzorSt");
        senzorDreapta.enableLed(false);
        senzorStanga.enableLed(false);
        senzorDistanta = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "senzorDistanta");
        senzorDistantaRev = hardwareMap.get(Rev2mDistanceSensor.class, "senzorDistantaRev");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while (imu.isGyroCalibrated());

//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
    }

    public void runMotors (double sF, double sS, double dF, double dS) {
        stangaFata.setPower(sF);
        stangaSpate.setPower(sS);
        dreaptaSpate.setPower(dS);
        dreaptaFata.setPower(dF);
    }

    public void forward(double power) {


        stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stangaFata.setPower(-power);
        stangaSpate.setPower(-power);
        dreaptaFata.setPower(power);
        dreaptaSpate.setPower(power);
    }

    public void zeroPowerBeh() {
        stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int conversieCmToTick(double distance){
        // return (int) (distance/(3529/1000)) + 100;
        return (int) (distance * 3.559);
    }

    public int cmToTicks (double distance) {
        //Distance is in cm
        double wheelDiameter = 10, gearRatio = 2, motorTicks = 1120;
        return (int) (distance / (wheelDiameter * Math.PI) * motorTicks * gearRatio);
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

        stangaFata.setPower(power);
        stangaSpate.setPower(power);
        dreaptaFata.setPower(power);
        dreaptaSpate.setPower(power);

        runtime.reset();

        while (stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout);

        stopDriving();

    }

    public void macaraDreaptaEncoder (int ticks, double power, int timeout) {

        //ticks < 0: ridicare
        //ticks > 0: coborare
        ElapsedTime runtime = new ElapsedTime();

        scripeteDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scripeteDreapta.setTargetPosition(ticks);

        scripeteDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        scripeteDreapta.setPower(power);

        runtime.reset();
        while (scripeteDreapta.isBusy() && runtime.seconds() < timeout);

        scripeteDreapta.setPower(0);

    }

    public void macaraStangaEncoder (int ticks, double power, int timeout) {

        //ticks < 0: ridicare
        //ticks > 0: coborare
        ElapsedTime runtime = new ElapsedTime();

        scripeteStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scripeteStanga.setTargetPosition(ticks);

        scripeteStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        scripeteStanga.setPower(power);

        runtime.reset();
        while (scripeteStanga.isBusy() && runtime.seconds() < timeout);

        scripeteStanga.setPower(0);
    }

    public void macaraStDrEnc (int ticks, double power, int timeout) {

        ElapsedTime runtime = new ElapsedTime();

        scripeteDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scripeteStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scripeteStanga.setTargetPosition(ticks);
        scripeteDreapta.setTargetPosition(ticks);

        scripeteDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scripeteStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        scripeteStanga.setPower(power);
        scripeteDreapta.setPower(power);

        runtime.reset();
        while (scripeteDreapta.isBusy() && scripeteStanga.isBusy() && runtime.seconds() < timeout);

        scripeteStanga.setPower(0);
        scripeteDreapta.setPower(0);

    }


    public void strafeCorrectionTest(int distance, double power, int timeout, Telemetry telemetry){
        //Distance > 0 => dreapta
        //Distance < 0 => stanga

        ElapsedTime runtime = new ElapsedTime();

        correction = checkDirection();
        correction = Math.toRadians(correction) / 5;

        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaFata.setTargetPosition(-distance);
        stangaSpate.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(-distance);
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

        while (stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout) {
            correction = checkDirection();
            correction = Math.toRadians(correction);

            stangaSpate .setPower(power  + correction);
            stangaFata  .setPower(power  - correction);
            dreaptaSpate.setPower(power  - correction);
            dreaptaFata .setPower(power  + correction);

            telemetry.addData("Correction: ", correction);
            telemetry.addData("Stanga fata: ", stangaFata.getCurrentPosition());
            telemetry.addData("Stanga spate: ", stangaSpate.getCurrentPosition());
            telemetry.addData("Dreapta fata: ", dreaptaFata.getCurrentPosition());
            telemetry.addData("Dreapta spate", dreaptaSpate.getCurrentPosition());
            telemetry.update();
        }

        stopDriving();
    }

    public void strafe (int distance, double power, int timeout) {

        //Distance > 0 => dreapta
        //Distance < 0 => stanga

        ElapsedTime runtime = new ElapsedTime();

        correction = checkDirection();

        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaFata.setTargetPosition(-distance);
        stangaSpate.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(-distance);
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

        while (stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout) {
            correction = checkDirection();
            correction = Math.toRadians(correction);

            stangaSpate .setPower(power  + correction);
            stangaFata  .setPower(power  - correction);
            dreaptaSpate.setPower(power  - correction);
            dreaptaFata .setPower(power  + correction);
        }

        stopDriving();
    }


    public void stopDriving() {
        stangaFata.setPower(0);
        stangaSpate.setPower(0);
        dreaptaFata.setPower(0);
        dreaptaSpate.setPower(0);
    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void newAngle() {
        prevAngle = getAngle();
    }

    public double maintainAngle() {
        double correction, angle;

        angle = getAngle();

        if (angle == prevAngle) correction = 0;
        else {
            correction = prevAngle - angle;
        }

        return correction;
    }

    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 1;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    public void rotate(int degrees, double power, int timeout)
    {
        double  leftPower, rightPower;

        stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ElapsedTime runtime = new ElapsedTime();

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
            power = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        stangaFata.setPower(power);
        stangaSpate.setPower(power);
        dreaptaSpate.setPower(power);
        dreaptaFata.setPower(power);
//        leftMotor.setPower(leftPower);
//        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        stopDriving();
//        rightMotor.setPower(0);
//        leftMotor.setPower(0);

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAngle();
    }

}
