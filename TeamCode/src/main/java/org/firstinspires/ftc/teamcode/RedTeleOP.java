package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOP Rosu")
public class RedTeleOP extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap);
        waitForStart();

        robot.ghearaDreapta.setPosition(1);

        while (opModeIsActive()) {

            runtime.reset();

            /** GAMEPAD 1 */

            //Miscarea sasiului
//            double rotate = -gamepad1.right_stick_x;
//            double strafe =  gamepad1.left_stick_x;
//            double forward = gamepad1.left_stick_y;

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double direction = Math.atan2(x, y) - Math.toRadians(robot.getAngle());
            double ipotenuse = Math.sqrt(x * x + y * y);
            double rotate  = gamepad1.right_stick_x * 0.6;

//            if (rotate != 0) robot.newAngle();
//            double correction = robot.maintainAngle();
//            correction = Math.toRadians(correction);

            double strafe  = Math.sin(direction) * ipotenuse * 0.6;
            double forward = Math.cos(direction) * ipotenuse * 0.9;

            double sS = -strafe - forward - rotate;
            double dF =  strafe + forward - rotate;
            double sF =  strafe - forward - rotate;
            double dS = -strafe + forward - rotate;


            robot.stangaSpate.setPower(sS);
            robot.dreaptaFata.setPower(dF);
            robot.stangaFata.setPower(sF);
            robot.dreaptaSpate.setPower(dS);



            //Ridicarea scripetelui
            //robot.ridicareBratStanga.setPower(-gamepad1.right_stick_y * 0.9);


            //Ridicarea scripetelui din dreapta
            double raisePower = 0.8;
            int ticks = 1440;
            if (gamepad1.left_trigger == 1) {
                robot.scripeteDreapta.setPower(raisePower);
            }
            else if (gamepad1.left_bumper) {
                robot.scripeteDreapta.setPower(-raisePower);
            }
            else if (!gamepad2.right_bumper && gamepad2.right_trigger != 1) robot.scripeteDreapta.setPower(0);


            //Prinderea mineralului
            if (gamepad1.dpad_down) {
                robot.ghearaDreapta.setPosition(0.5);
            }
            else if (gamepad1.dpad_up) {
                robot.ghearaDreapta.setPosition(1);
            }

            //Ridicarea scripetelui din stanga
            if (gamepad1.right_bumper) {
                robot.scripeteStanga.setPower(-raisePower);
            }
            else if (gamepad1.right_trigger == 1) {
                robot.scripeteStanga.setPower(raisePower * robot.coborareScripete);
            }
            else if (gamepad2.left_trigger != 1 && !gamepad2.left_bumper) robot.scripeteStanga.setPower(0);


            //Prinderea capstone-ului
            if (gamepad1.a) {
                robot.ghearaStanga.setPosition(0.5);
            }
            if (gamepad1.y) {
                robot.ghearaStanga.setPosition(0);
            }

            //Resetare unghi
            if (gamepad1.x){
                robot.resetAngle();
            }


            /** GAMEPAD 2 */


            //Bascularea bratelor
            robot.ridicareBratDreapta.setPower(-gamepad2.right_stick_y * 0.9);
            robot.ridicareBratStanga.setPower(-gamepad2.left_stick_y * 0.9);

            //Ridicarea scripetelui pentru bratul din dreapta
            if (gamepad2.right_trigger == 1) {
                robot.scripeteDreapta.setPower(raisePower);
            }
            else if (gamepad2.right_bumper) {
                robot.scripeteDreapta.setPower(-raisePower);
            }
            else if (gamepad1.left_trigger != 1 && !gamepad1.left_bumper) robot.scripeteDreapta.setPower(0);

            //Ridicarea scripetelui pentru bratul din stanga
            if (gamepad2.left_bumper) {
                robot.scripeteStanga.setPower(-raisePower);
            }
            else if (gamepad2.left_trigger == 1) {
                robot.scripeteStanga.setPower(raisePower);
            }
            else if (gamepad1.right_trigger != 1 && !gamepad1.right_bumper) robot.scripeteStanga.setPower(0);


//            //Prinderea mineralului pentru bratul din dreapta
//            if (gamepad2.a) {
//                robot.ghearaDreapta.setPosition(0.5);
//            }
//            else if (gamepad2.y) {
//                robot.ghearaDreapta.setPosition(1);
//            }
//            //Prinderea de capstone
//            if (gamepad2.dpad_up) {
//                robot.ghearaStanga.setPosition(0);
//            }
//            else if (gamepad2.dpad_down) {
//                robot.ghearaStanga.setPosition(0.5);
//            }

        }

    }
}
