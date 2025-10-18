package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Shooterv2;

@Config
@TeleOp(name="TransfertoShooterTest", group="Testing")
public class TransfertoShooterTest extends LinearOpMode {

    public static double motorPower = 0.85;
    public static double motorfeedpower = 0.7;
    public static double RPM = 2000;
    public static int detectionDistance = 3;
    public static int loadDistance = 200;

    DcMotor intakeMotor;
    DcMotor transferMotor;

    Shooterv2 shooter;

    public DistanceSensor transferSensor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        shooter = new Shooterv2(hardwareMap);
        transferSensor = hardwareMap.get(DistanceSensor.class, "rightTransferSensor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean shooterOn = false;
        boolean transferOn = false;
        boolean intakeOn = false;
        boolean load = false;
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1 = currentGamepad1;
            currentGamepad1 = gamepad1;
            shooter.targetRPM = RPM;

            if (gamepad1.a) {
                if (transferSensor.getDistance(DistanceUnit.CM) >= detectionDistance) {
                    transferOn = true;
                }
                if (!transferOn) {
                    intakeOn = !intakeOn;
                }

            }

            if (transferOn) {
                intakeMotor.setPower(motorPower);
                transferMotor.setPower(motorPower);
                intakeOn = true;

                if (transferSensor.getDistance(DistanceUnit.CM) <= detectionDistance) {
                    transferOn = false;
                    transferMotor.setPower(0);
                }
            } else if (intakeOn) {
                intakeMotor.setPower(motorPower);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad1.b) {
                shooterOn = !shooterOn;
            }

            if (shooterOn) {
                shooter.updateBang();
            } else {
                shooter.setPower(0);
            }

            if (gamepad1.y && Shooterv2.RPMInThreshold(shooter.leftRPM, shooter.rightRPM, shooter.targetRPM)) {
                transferMotor.setPower(motorfeedpower);
                intakeMotor.setPower(motorfeedpower);
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
            telemetry.addData("leftPower", shooter.leftShooterMotor.getPower());
            telemetry.addData("rightPower", shooter.rightShooterMotor.getPower());
            telemetry.addData("shooterOn", shooterOn);
            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("transferOn", transferOn);
            telemetry.addData("load", load);
            telemetry.addData("transfer distance", transferSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}