package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="IntakeTest2", group="Testing")
public class TransfertoShooterTest extends LinearOpMode {

    public static double motorPower = 0.5;
    public static double shootermotorPower = 0.5;
    public static int detectionDistance = 5;

    DcMotor intakeMotor;
    DcMotor transferMotor;
    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    public DistanceSensor transferSensor;

    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        leftShooterMotor = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooterMotor = hardwareMap.get(DcMotor.class, "rightShooter");
        transferSensor = hardwareMap.get(DistanceSensor.class, "transferSensor");
        ElapsedTime time = new ElapsedTime();
        double shootTime = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            time.reset();
            if (gamepad1.a) {
                while (transferSensor.getDistance(DistanceUnit.CM) >= detectionDistance) {
                    intakeMotor.setPower(motorPower);
                    transferMotor.setPower(-motorPower);
                }
                intakeMotor.setPower(0);
            }
            if (gamepad1.b) {
                leftShooterMotor.setPower(shootermotorPower);
                rightShooterMotor.setPower(-shootermotorPower);
                shootTime = 0;
            }
            if (gamepad1.y && shootTime >= 1000) {
                intakeMotor.setPower(motorPower);
            }

            shootTime += time.milliseconds();


        }

    }
}
