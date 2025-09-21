package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Camera {

    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;

    public Camera() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(HWMap.get(WebcamName.class, "Webcam"))
            // prob need to change
            .setCameraResolution(new Size(640, 480))
            .build();
    }

    public int findID() {
        AprilTagDetection tag;
        if(!tagProcessor.getDetections().isEmpty()) {
            tag = tagProcessor.getDetections().get(0);
            return tag.metadata.id;
        }
        return -1;
    }

}
