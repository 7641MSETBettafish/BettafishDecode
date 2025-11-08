package org.firstinspires.ftc.teamcode.testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.*;

@Config
@TeleOp(name="AutoAlign", group="Testing")
public class AutoAlign extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public static double hPmin = 0.01;
    public static double hPmax = 0.08;
    public static double hDeadZone = 5;
    public static double hSpeed = 0.1;

    public static double pPmin = 0.01;
    public static double pPmax = 0.08;
    public static double pDeadZone = 2;

    final Pose2d goalPosition = new Pose2d(-60, 63, 0);

    MecanumDrive drive;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1421.04,1421.04,649.331,357.761)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .build();


        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        boolean angleHold = false;
        boolean positionHold = false;
        Pose2d holdPose = null;

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            if (!tagProcessor.getDetections().isEmpty()) {

                AprilTagDetection tag;
                tag = tagProcessor.getDetections().get(0);

                if (currentGamepad1.a && !previousGamepad1.a) angleHold = !angleHold;
                if (currentGamepad1.b && !previousGamepad1.b) {
                    positionHold = !positionHold;
                    if (positionHold) holdPose = drive.localizer.getPose();
                }

                drive.updatePoseEstimate();
                Pose2d pose = drive.localizer.getPose();

                if (angleHold) {
                /*
                theta = 36 - tag.ftcPose.bearing + tag.ftcPose.yaw;
                    x = tag.ftcPose.range * Math.cos(Math.toRadians(theta)) + 11.3;
                    y = tag.ftcPose.range * Math.sin(Math.toRadians(theta)) + 11.25;
                 */
                    double dxGoal = goalPosition.position.x - pose.position.x;
                    double dyGoal = goalPosition.position.y - pose.position.y;
                    double targetAngle = 0;
                    double headingError = tag.ftcPose.bearing;

                    double rxCmd = hSpeed * headingError;
                    rxCmd = Math.max((-1*hPmax), Math.min(hPmax, rxCmd));

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

                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;

                dash.sendTelemetryPacket(packet);

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
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double LFPower = (y + x + rx) / denominator;
            double LBPower = (y - x + rx) / denominator;
            double RFPower = (y - x - rx) / denominator;
            double RBPower = (y + x - rx) / denominator;

            drive.leftFront.setPower(LFPower);
            drive.leftBack.setPower(LBPower);
            drive.rightFront.setPower(RFPower);
            drive.rightBack.setPower(RBPower);

        }
    }
}
