package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Auton.Auto2025;


public class Camera {

    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;


    public Camera(HardwareMap HWMap) {
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

    public class findID implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            AprilTagDetection tag;
            while (tagProcessor.getDetections().isEmpty()) {

            }
            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);
                Auto2025.tagID = tag.metadata.id;
                return false;
            }
        }
    }

    public Action findID(){
        return new findID();
    }

}
