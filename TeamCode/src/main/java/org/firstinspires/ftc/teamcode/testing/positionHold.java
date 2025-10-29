package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class positionHold extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    // PID constants
    public static double xkP = 0.05;
    public static double xkI= 0.0;
    public static double xkD = 0.003;

    public static double ykP = 0.05;
    public static double ykI= 0.0;
    public static double ykD = 0.003;

    public static double deadZone = 1.5;  // joystick & PID deadzone

    MecanumDrive drive;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        boolean positionHold = false;
        Pose2d holdPose = null;

        // PID variables
        double xIntegral = 0;
        double yIntegral = 0;
        double lastXError = 0;
        double lastYError = 0;
        double lastTime = 0;

        waitForStart();
        lastTime = getRuntime();

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double y = gamepad1.left_stick_y; // reversed direction
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Toggle position hold
            if (currentGamepad1.b && !previousGamepad1.b) {
                positionHold = !positionHold;
                if (positionHold) holdPose = drive.localizer.getPose();

                // Reset PID
                xIntegral = 0;
                yIntegral = 0;
                lastXError = 0;
                lastYError = 0;
            }

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            // Time step for PID
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            if (positionHold && holdPose != null) {
                double xError = holdPose.position.x - pose.position.x;
                double yError = holdPose.position.y - pose.position.y;

                // Apply deadzone to PID error
                if (Math.abs(xError) < deadZone) xError = 0;
                if (Math.abs(yError) < deadZone) yError = 0;

                xIntegral += xError * dt;
                yIntegral += yError * dt;

                double xDerivative = (xError - lastXError) / dt;
                double yDerivative = (yError - lastYError) / dt;

                lastXError = xError;
                lastYError = yError;

                double xOutput = (xkP * xError) + (xkI * xIntegral) + (xkD * xDerivative);
                double yOutput = (ykP * yError) + (ykI * yIntegral) + (ykD * yDerivative);

                // Field-centric transformation
                double heading = pose.heading.toDouble();
                double xRobot = xOutput * Math.cos(-heading) - yOutput * Math.sin(-heading);
                double yRobot = xOutput * Math.sin(-heading) + yOutput * Math.cos(-heading);

                x = xRobot;
                y = yRobot;

                // Apply deadzone to PID outputs
                if (Math.abs(x) < deadZone) x = 0;
                if (Math.abs(y) < deadZone) y = 0;
            }


            // Mecanum drive calculation
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double LFPower = (y + x + rx) / denominator;
            double LBPower = (y - x + rx) / denominator;
            double RFPower = (y - x - rx) / denominator;
            double RBPower = (y + x - rx) / denominator;

            drive.leftFront.setPower(LFPower);
            drive.leftBack.setPower(LBPower);
            drive.rightFront.setPower(RFPower);
            drive.rightBack.setPower(RBPower);

            // Telemetry
            telemetry.addData("Position Hold (B)", positionHold);
            if (holdPose != null) {
                telemetry.addData("Hold X", holdPose.position.x);
                telemetry.addData("Hold Y", holdPose.position.y);
            }
            telemetry.addData("Robot X", pose.position.x);
            telemetry.addData("Robot Y", pose.position.y);
            telemetry.addData("Robot H", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
