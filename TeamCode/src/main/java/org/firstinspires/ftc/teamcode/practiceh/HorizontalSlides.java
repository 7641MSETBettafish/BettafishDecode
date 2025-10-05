package org.firstinspires.ftc.teamcode.practiceh;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class HorizontalSlides {

    private Servo rightSlidesServo;
    private Servo leftSlidesServo;

    public HorizontalSlides(HardwareMap hardwareMap) {
        rightSlidesServo = hardwareMap.get(Servo.class, "rightSlidesServo");
        leftSlidesServo = hardwareMap.get(Servo.class,"leftSlidesServo");
    }

    public class ExtendSlides implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rightSlidesServo.setPosition(0.7);
            leftSlidesServo.setPosition(0.7);

            return false;
        }

    }

    public class RetractSlides implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rightSlidesServo.setPosition(0);
            leftSlidesServo.setPosition(0);

            return false;
        }

    }
}
