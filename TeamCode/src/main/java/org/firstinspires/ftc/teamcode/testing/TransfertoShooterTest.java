package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name="IntakeTest2", group="Testing")
public class TransfertoShooterTest extends LinearOpMode {
    public static double startX = 0;
    public static double startY = 0;
    public static double startH = 0;

    public static double motorPower = 0.5;
    public static double shootermotorPower = 0.5;
    public static int detectionDistance = 5;
    public static int loadDistance = 200;

    DcMotor intakeMotor;
    DcMotor transferMotor;
    MecanumDrive drive;
    Shooter shooter;

    public DistanceSensor transferSensor;


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startH));
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

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double LFPower = (y + x + rx) / denominator;
            double LBPower = (y - x + rx) / denominator;
            double RFPower = (y - x - rx) / denominator;
            double RBPower = (y + x - rx) / denominator;

            drive.leftFront.setPower(LFPower);
            drive.leftBack.setPower(LBPower);
            drive.rightFront.setPower(RFPower);
            drive.rightBack.setPower(RBPower);

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
