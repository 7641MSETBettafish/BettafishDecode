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
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Shooterv2;

@Config
@TeleOp(name="ShooterTest", group="Testing")
public class ShooterTest extends LinearOpMode {

    public static double RPM = 0;

    Shooterv2 shooter;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooterv2(hardwareMap);

        double lastLeftPosition = 0;
        double lastRightPosition = 0;
        ElapsedTime time = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            shooter.updateRPM();
            shooter.targetRPM = RPM;
            shooter.updateBang();

            telemetry.addData("leftRPM", shooter.leftRPM);
            telemetry.addData("rightRPM", shooter.rightRPM);
            telemetry.addData("targetRPM", shooter.targetRPM);
            telemetry.addData("leftPower", shooter.leftShooterMotor.getPower());
            telemetry.addData("rightPower", shooter.rightShooterMotor.getPower());
            telemetry.addData("targetRPM", shooter.targetRPM);
            telemetry.addData("time change", time.milliseconds());
            telemetry.update();

            time.reset();
        }

    }
}
