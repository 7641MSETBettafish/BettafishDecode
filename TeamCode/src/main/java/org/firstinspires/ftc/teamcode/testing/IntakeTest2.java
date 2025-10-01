package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@Config
@TeleOp(name="IntakeTest2", group="Testing")
public class IntakeTest2 extends LinearOpMode {

    public static double motorPower = 0.5;
    public static int detectionDistance = 5;

    DcMotor intakeMotor;
    DcMotor transferMotor;

    public DistanceSensor transferSensor;

    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeShooter");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        //transferSensor = hardwareMap.get(DistanceSensor.class, "transferSensor");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            transferMotor.setPower(-motorPower);
            intakeMotor.setPower(motorPower);

            /*if (transferSensor.getDistance(DistanceUnit.CM) <= detectionDistance) {
                transferMotor.setPower(0);
            } else {
                intakeMotor.setPower(motorPower);
                transferMotor.setPower(-motorPower);
            }*/

        }

    }
}
