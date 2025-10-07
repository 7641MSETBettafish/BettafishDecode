package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name="IntakeTest2", group="Testing")
public class TransfertoShooterTest extends LinearOpMode {

    public static double motorPower = 0.5;
    public static double shootermotorPower = 0.5;
    public static int detectionDistance = 5;
    public static int loadDistance = 200;

    DcMotor intakeMotor;
    DcMotor transferMotor;
    Shooter shooter;

    public DistanceSensor transferSensor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        shooter = new Shooter(hardwareMap);
        transferSensor = hardwareMap.get(DistanceSensor.class, "transferSensor");

        boolean shooterOn = false;
        boolean intakeOn = false;
        boolean load = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (transferSensor.getDistance(DistanceUnit.CM) >= detectionDistance) {
                    intakeOn = true;
                }
            }

            if (intakeOn) {
                intakeMotor.setPower(motorPower);
                transferMotor.setPower(-motorPower);

                if (transferSensor.getDistance(DistanceUnit.CM) <= detectionDistance) {
                    intakeOn = false;
                    transferMotor.setPower(0);
                }
            }

            if (gamepad1.b) {
                if (shooterOn) {
                    shooter.setPower(0);
                } else {
                    shooter.setPower(shootermotorPower);
                }
                shooterOn = !shooterOn;

            }

            if (gamepad1.y && Shooter.RPMInThreshold(shooter.leftRPM, shooter.rightRPM, shootermotorPower)) {
                transferMotor.setPower(motorPower);
                load = true;
                transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (load) {
                if (transferMotor.getCurrentPosition() >= loadDistance) {
                    transferMotor.setPower(0);
                    load = false;
                }
            }

            shooter.updateRPM();

            telemetry.addData("leftRPM", shooter.leftRPM);
            telemetry.addData("rightRPM", shooter.rightRPM);
            telemetry.addData("shooterOn", shooterOn);
            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("load", load);
            telemetry.addData("transfer distance", transferSensor.getDistance(DistanceUnit.CM));
        }

    }
}
