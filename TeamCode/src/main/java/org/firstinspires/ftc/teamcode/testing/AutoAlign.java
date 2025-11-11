package org.firstinspires.ftc.teamcode.testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
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

    public static double hSpeed = 0.023;

    final Pose2d goalPosition = new Pose2d(-60, 63, 0);

    public PIDController PID = new PIDController(hSpeed,0,0);
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

            PID.setPID(hSpeed,0, 0);

            if (currentGamepad1.a && !previousGamepad1.a) angleHold = !angleHold;

            AprilTagDetection tag;
            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);

                if (tag.id == 20 || tag.id == 24) {

                    drive.updatePoseEstimate();
                    Pose2d pose = drive.localizer.getPose();

                    if (angleHold) {
                        rx = PID.calculate(tag.ftcPose.bearing, 12);
                        ;
                    }

                    telemetry.addData("Angle Hold (A)", angleHold);
                    telemetry.addData("Position Hold (B)", positionHold);
                    if (holdPose != null) {
                        telemetry.addData("Hold X", holdPose.position.x);
                        telemetry.addData("Hold Y", holdPose.position.y);
                    }
                    telemetry.addData("Robot X", pose.position.x);
                    telemetry.addData("Robot Y", pose.position.y);
                    telemetry.addData("Robot H", Math.toDegrees(pose.heading.toDouble()));
                    telemetry.addData("id", tag.metadata.id);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);
                    telemetry.addData("bearing", tag.ftcPose.bearing);
                    telemetry.addData("elevation", tag.ftcPose.elevation);
                    telemetry.addData("range", tag.ftcPose.range);
                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                    telemetry.update();
                }
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
