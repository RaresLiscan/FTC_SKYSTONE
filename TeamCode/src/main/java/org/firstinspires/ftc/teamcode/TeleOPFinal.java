package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleOP")
@Disabled
public class TeleOPFinal extends LinearOpMode {

    private RobotMap robot = null;
    private double DELAY = 0.2;

    private double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap, this);
        waitForStart();

        robot.ghearaStanga.setPosition(0);

        runtime.reset();

        while (opModeIsActive()) {

            /** GAMEPAD 1 */

            //Miscarea sasiului

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double direction = Math.atan2(x, y) - Math.toRadians(robot.getAngle()) + angle;
            double ipotenuse = Math.sqrt(x * x + y * y);
            double rotate  = gamepad1.right_stick_x * 0.65;
            double strafe  = Math.sin(direction) * ipotenuse;
            double forward = Math.cos(direction) * ipotenuse;


            double sS = robot.SQRT(-strafe - forward - rotate);
            double dF = robot.SQRT(strafe + forward - rotate);
            double sF = robot.SQRT(strafe - forward - rotate);
            double dS = robot.SQRT(-strafe + forward - rotate);

            robot.stangaSpate.setPower(sS);
            robot.dreaptaFata.setPower(dF);
            robot.stangaFata.setPower(sF);
            robot.dreaptaSpate.setPower(dS);



            //Ridicarea scripetelui din dreapta
            double raisePower = 0.8;
            int ticks = 1440;
            if (gamepad1.right_trigger != 0) {
                robot.scripeteDreapta.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.right_bumper) {
                robot.scripeteDreapta.setPower(-raisePower);
            }
            else if (gamepad2.right_trigger == 0 && !gamepad2.right_bumper) robot.scripeteDreapta.setPower(0);

            //Ridicarea scripetelui din stanga
            if (gamepad1.left_bumper) {
                robot.scripeteStanga.setPower(-raisePower);
            }
            else if (gamepad1.left_trigger != 0) {
                robot.scripeteStanga.setPower(gamepad1.left_trigger);
            }
            else if (gamepad2.left_trigger == 0 && !gamepad2.left_bumper) robot.scripeteStanga.setPower(0);



            //Prinderea mineralului
            if (gamepad1.right_stick_button && runtime.seconds() >= DELAY) {
                if (robot.ghearaDreapta.getPosition() == 0.5) {
                    robot.ghearaDreapta.setPosition(1);
                }
                else {
                    robot.ghearaDreapta.setPosition(0.5);
                }
                runtime.reset();
            }
            else if (gamepad1.left_stick_button && runtime.seconds() >= DELAY) {
                if (robot.ghearaStanga.getPosition() == 0.5) {
                    robot.ghearaStanga.setPosition(0);
                }
                else {
                    robot.ghearaStanga.setPosition(0.5);
                }
                runtime.reset();
            }

            //Resetare unghi
//            if (gamepad1.x){
//                robot.resetAngle();
//            }
            if (gamepad1.dpad_left) {
                angle = Math.PI / 2;
            }
            if (gamepad1.dpad_right) {
                angle = -Math.PI / 2;
            }
            if (gamepad1.dpad_up) {
                angle = 0;
            }

            if (gamepad1.b) {
                robot.ghearaDreapta.setPosition(0);
            }

            /** GAMEPAD 2 */

            //Bascularea bratelor
            robot.ridicareBratDreapta.setPower(-gamepad2.right_stick_y * 0.9);
            robot.ridicareBratStanga.setPower(-gamepad2.left_stick_y * 0.9);

            //Ridicarea scripetelui pentru bratul din dreapta
            if (gamepad2.right_trigger != 0) {
                robot.scripeteDreapta.setPower(gamepad2.right_trigger);
            }
            else if (gamepad2.right_bumper) {
                robot.scripeteDreapta.setPower(-raisePower);
            }
            else if (gamepad1.right_trigger == 0 && !gamepad1.right_bumper) robot.scripeteDreapta.setPower(0);

            //Ridicarea scripetelui pentru bratul din stanga
            if (gamepad2.left_bumper) {
                robot.scripeteStanga.setPower(-raisePower);
            }
            else if (gamepad2.left_trigger != 0) {
//                robot.scripeteStanga.setPower(-raisePower);
                robot.scripeteStanga.setPower(gamepad2.left_trigger);
            }
            else if (gamepad1.left_trigger == 0 && !gamepad1.left_bumper) robot.scripeteStanga.setPower(0);

            if (gamepad2.x) {
                robot.macaraStangaEncoder(-110, 0.9, 2);
                robot.macaraDreaptaEncoder(-110, 0.9, 2);
            }

        }

    }
}
