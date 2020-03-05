package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test senzori de distanta")
public class SenzoriDistantaTest extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Senzor fata stanga: ", robot.senzorFataStanga.getDistance(DistanceUnit.CM));
            telemetry.addData("Senzor fata dreapta: ", robot.senzorFataDreapta.getDistance(DistanceUnit.CM));
            telemetry.addData("Senzor laterala stanga: ", robot.senzorLateralaStanga.getDistance(DistanceUnit.CM));
            telemetry.addData("Senzor laterala dreapta: ", robot.senzorLateralaDreapta.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }
}
