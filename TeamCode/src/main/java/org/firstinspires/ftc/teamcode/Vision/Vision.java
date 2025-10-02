package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class Vision extends LinearOpMode {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(57, 60.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        // inches
        double z = 29.5 - 18.5;
        double x;
        double y;

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            AprilTagDetection tag;
            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);

                if(tag.metadata != null) {

                    x = z * Math.sin(Math.toRadians(tag.ftcPose.bearing + 180 - Math.toDegrees(drive.localizer.getPose().heading.toDouble()))) / Math.tan(Math.toRadians(tag.ftcPose.elevation));
                    y = z * Math.cos(Math.toRadians(tag.ftcPose.bearing + 180 - Math.toDegrees(drive.localizer.getPose().heading.toDouble()))) / Math.tan(Math.toRadians(tag.ftcPose.elevation));

                    telemetry.addData("id", tag.metadata.id);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);
                    telemetry.addData("bearing", tag.ftcPose.bearing);
                    telemetry.addData("elevation", tag.ftcPose.elevation);
                    telemetry.addData("Expected X", x);
                    telemetry.addData("Expected Y", y);
                }
            }
            telemetry.addData("robot X", drive.localizer.getPose().position.x);
            telemetry.addData("robot Y", drive.localizer.getPose().position.y);
            telemetry.addData("robot H", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            drive.updatePoseEstimate();
            telemetry.update();
        }

    }
}
