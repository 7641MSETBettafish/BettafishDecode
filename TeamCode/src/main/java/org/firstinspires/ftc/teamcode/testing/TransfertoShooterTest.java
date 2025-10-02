package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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
        //transferSensor = hardwareMap.get(DistanceSensor.class, "transferSensor");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            transferMotor.setPower(-motorPower);
            intakeMotor.setPower(motorPower);

            leftShooterMotor.setPower(shootermotorPower);
            rightShooterMotor.setPower(-shootermotorPower);

            /*if (transferSensor.getDistance(DistanceUnit.CM) <= detectionDistance) {
                transferMotor.setPower(0);
            } else {
                intakeMotor.setPower(motorPower);
                transferMotor.setPower(-motorPower);
            }*/

        }

    }
}
