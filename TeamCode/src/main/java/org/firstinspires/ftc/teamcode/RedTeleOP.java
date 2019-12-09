package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="TeleOP Rosu")
@Disabled
public class RedTeleOP extends LinearOpMode {

    RobotMap robot = null;

    private void basculare (int ticks, double power, DcMotor motor) {
        if (ticks < 0) motor.setPower(-power);
        else motor.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            /** GAMEPAD 1 */
            //Miscarea sasiului
            double rotate = -gamepad1.right_stick_x;
            double strafe =  gamepad1.left_stick_x;
            double forward = gamepad1.left_stick_y;

            double sS = -strafe - forward - rotate;
            double dF =  strafe + forward - rotate;
            double sF =  strafe - forward - rotate;
            double dS = -strafe + forward - rotate;

            robot.stangaSpate.setPower(sS);
            robot.dreaptaFata.setPower(dF);
            robot.stangaFata.setPower(sF);
            robot.dreaptaSpate.setPower(dS);

            //Ridicarea scripetelui
            robot.scripeteDreapta.setPower(gamepad1.right_stick_y * 0.7);


            //Prinderea mineralului
            if (gamepad1.right_trigger != 0) {
                robot.ghearaDreapta.setPosition(1);
            }
            if (gamepad1.right_bumper) {
                robot.ghearaDreapta.setPosition(0.5);
            }

            //Eliberare capstone
            if (gamepad1.left_trigger != 0) {
                robot.ghearaStanga.setPosition(0);
            }
            if (gamepad1.left_bumper) {
                robot.ghearaStanga.setPosition(0.5);
            }


            //Bascularea bratului
            double raisePower = 0.4;
            int ticks = 1440;
            //Pentru bratul din dreapta
            if (gamepad1.a) {
                basculare(-ticks, raisePower, robot.ridicareBratDreapta);
            }
            else if (gamepad1.y) {
                basculare(ticks, raisePower, robot.ridicareBratDreapta);
            }
            else robot.ridicareBratDreapta.setPower(0);

            //Pentru bratul din stanga
            if (gamepad1.dpad_up) {
                basculare(ticks, raisePower, robot.ridicareBratStanga);
            }
            else if (gamepad1.dpad_down) {
                basculare(-ticks, raisePower, robot.ridicareBratStanga);
            }
            else robot.ridicareBratStanga.setPower(0);


            /** GAMEPAD 2 */


            //Ridicare scripete dreapta
            robot.scripeteDreapta.setPower(gamepad2.right_stick_y * 0.7);
            //Ridicare scripete stanga
            robot.scripeteStanga.setPower(gamepad2.left_stick_y * 0.7);


            //Bascularea bratului
            double powerRise = 0.4;
            //Pentru bratul din dreapta
            if (gamepad2.a) {
                basculare(-ticks, powerRise, robot.ridicareBratDreapta);
            }
            else if (gamepad2.y) {
                basculare(ticks, powerRise, robot.ridicareBratDreapta);
            }
            else robot.ridicareBratDreapta.setPower(0);
            //Pentru bratul din stanga
            if (gamepad2.dpad_up) {
                basculare(ticks, powerRise, robot.ridicareBratStanga);
            }
            else if (gamepad2.dpad_down) {
                basculare(-ticks, powerRise, robot.ridicareBratStanga);
            }
            else robot.ridicareBratStanga.setPower(0);

            //Prinderea mineralului pentru bratul din dreapta
            if (gamepad2.right_bumper) {
                robot.ghearaDreapta.setPosition(0.5);
            }
            else if (gamepad2.right_trigger != 0) {
                robot.ghearaDreapta.setPosition(1);
            }
            //Prinderea de capstone
            if (gamepad2.left_trigger != 0) {
                robot.ghearaStanga.setPosition(0);
            }
            else if (gamepad2.left_bumper) {
                robot.ghearaStanga.setPosition(0.5);
            }

        }
    }
}
