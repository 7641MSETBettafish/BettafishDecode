package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            AprilTagDetection tag;
            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);
                if(tag.metadata != null) {
                    telemetry.addData("id", tag.metadata.id);
                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);
                }

            }

            telemetry.update();
        }
    }

    public int findID() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            return detections.get(0).metadata.id;
        }
        return -1;
    }
}
