package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="PunkOP")
public class TestTeleOP extends LinearOpMode {

    RobotMap robot = null;

    private void basculare (int ticks, double power, DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {


            //Miscarea sasiului
            double rotate = -gamepad1.left_stick_x * 0.45;
            double strafe = gamepad1.right_stick_x * 0.7;
            double forward = gamepad1.left_stick_y * 0.45;
            //double diagonal =

            double sS = -strafe + forward - rotate;
            double dF = strafe + forward  -rotate;
            double sF = -strafe + forward + rotate;
            double dS = strafe + forward + rotate;

            robot.stangaSpate.setPower(sS);
            robot.dreaptaFata.setPower(dF);
            robot.stangaFata.setPower(sF);
            robot.dreaptaSpate.setPower(dS);

            robot.scripeteStanga.setPower(gamepad2.left_stick_y * 0.7);
            robot.scripeteDreapta.setPower(gamepad2.right_stick_y * 0.7);


            //Prinderea mineralului
            if (gamepad2.right_bumper){
                robot.gheara.setPosition(1);
            }
            else if (gamepad2.left_bumper){
                robot.gheara.setPosition(0.5);
            }

            //Basculare bazata pe encodere
            double raisePower = 0.2;
            int ticks = 1440;
            //Pentru bratul din stanga
            if (gamepad2.dpad_down) {
                basculare(ticks, raisePower, robot.ridicareBratStanga);
            }
            else if (gamepad2.dpad_up) {
                basculare(-ticks, raisePower, robot.ridicareBratStanga);
            }
            if (robot.ridicareBratStanga.getPower() == 0 && robot.ridicareBratStanga.isBusy()) {
                robot.ridicareBratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //Pentru bratul din dreapta
            if (gamepad2.a) {
                basculare(-ticks, raisePower, robot.ridicareBratDreapta);
            }
            else if (gamepad2.y) {
                basculare(ticks, raisePower, robot.ridicareBratDreapta);
            }
            if (robot.ridicareBratDreapta.getPower() == 0 && robot.ridicareBratDreapta.isBusy()) {
                robot.ridicareBratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        }
    }
}
