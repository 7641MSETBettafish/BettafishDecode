package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.mechanisms.Shooter.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.mechanisms.Shooter.motorTicksPerRevolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name="ShooterTest", group="Testing")
public class ShooterTest extends LinearOpMode {

    public static double RPM = 0;

    Shooter shooter;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter(hardwareMap);

        ElapsedTime time = new ElapsedTime();
        double leftMaxRPM = 0;
        double rightMaxRPM = 0;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            shooter.updateRPM();
            shooter.targetRPM = RPM;
            shooter.updatePID();

            if (shooter.leftRPM > leftMaxRPM) {
                leftMaxRPM = shooter.leftRPM;
            }
            if (shooter.rightRPM > rightMaxRPM) {
                rightMaxRPM = shooter.rightRPM;
            }

            telemetry.addData("leftRPM", shooter.leftRPM);
            telemetry.addData("rightRPM", shooter.rightRPM);
            telemetry.addData("leftmaxrpm", leftMaxRPM);
            telemetry.addData("rightmaxrpm", rightMaxRPM);
            telemetry.addData("targetRPM", shooter.targetRPM);
            telemetry.addData("leftPower", shooter.leftShooterMotor.getPower());
            telemetry.addData("rightPower", shooter.rightShooterMotor.getPower());
            telemetry.addData("time change", time.milliseconds());
            telemetry.update();

            time.reset();
        }

    }
}
