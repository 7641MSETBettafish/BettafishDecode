package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp(name="ShooterTest", group="Testing")
public class ShooterTest extends LinearOpMode {

    public static double motorTicksPerDegree = 103.8;
    public static double motorPower = 0;
    public static double gearRatio = 1 / 1.5;

    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftShooterMotor = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooterMotor = hardwareMap.get(DcMotor.class, "rightShooter");

        leftShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double lastLeftPosition = 0;
        double lastRightPosition = 0;
        ElapsedTime time = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            time.reset();

            leftShooterMotor.setPower(motorPower);
            rightShooterMotor.setPower(motorPower);

            telemetry.addData("leftRPM", (leftShooterMotor.getCurrentPosition() - lastLeftPosition) / time.milliseconds() / motorTicksPerDegree / 360 * gearRatio * 360000);
            telemetry.addData("rightRPM", (rightShooterMotor.getCurrentPosition() - lastRightPosition) / time.milliseconds() / motorTicksPerDegree / 360 * gearRatio * 360000);
            telemetry.update();


            lastLeftPosition = leftShooterMotor.getCurrentPosition();
            lastRightPosition = rightShooterMotor.getCurrentPosition();
        }

    }
}
