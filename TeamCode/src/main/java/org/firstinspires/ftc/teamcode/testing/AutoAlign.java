package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp(name="AutoAlign", group="Testing")
public class AutoAlign extends LinearOpMode {

    public static double hPmin = 0.01;
    public static double hPmax = 0.08;
    public static double hDeadZone = 5;

    public static double pPmin = 0.01;
    public static double pPmax = 0.08;
    public static double pDeadZone = 2;

    final Pose2d goalPosition = new Pose2d(-60, 63, 0);

    MecanumDrive drive;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        boolean angleHold = false;
        boolean positionHold = false;
        Pose2d holdPose = null;

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (currentGamepad1.a && !previousGamepad1.a) angleHold = !angleHold;
            if (currentGamepad1.b && !previousGamepad1.b) {
                positionHold = !positionHold;
                if (positionHold) holdPose = drive.localizer.getPose();
            }

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            if (angleHold) {
                double dxGoal = goalPosition.position.x - pose.position.x;
                double dyGoal = goalPosition.position.y - pose.position.y;
                double targetAngle = Math.atan2(dyGoal, dxGoal);
                double headingError = targetAngle - pose.heading.toDouble();
                headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                double absErr = Math.abs(headingError);
                double kP_heading = hPmin + (hPmax - hPmin) * Math.min(absErr / Math.toRadians(45), 1.0);
                double rxCmd = kP_heading * headingError;
                rxCmd = Math.max(-1, Math.min(1, rxCmd));

                if (Math.abs(headingError) < Math.toRadians(hDeadZone)) rxCmd = 0;
                
                rx = rxCmd;
            }

            if (positionHold && holdPose != null) {
                double xError = holdPose.position.x - pose.position.x;
                double yError = holdPose.position.y - pose.position.y;
                double distanceError = Math.hypot(xError, yError);

                double kP_position = pPmin + (pPmax - pPmin) * Math.min(distanceError / 5.0, 1.0);
                if (Math.abs(xError) < pDeadZone) xError = 0;
                if (Math.abs(yError) < pDeadZone) yError = 0;
                x = kP_position * xError;
                y = kP_position * yError;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double LFPower = (y + x + rx) / denominator;
            double LBPower = (y - x + rx) / denominator;
            double RFPower = (y - x - rx) / denominator;
            double RBPower = (y + x - rx) / denominator;

            drive.leftFront.setPower(LFPower);
            drive.leftBack.setPower(LBPower);
            drive.rightFront.setPower(RFPower);
            drive.rightBack.setPower(RBPower);

            telemetry.addData("Angle Hold (A)", angleHold);
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
