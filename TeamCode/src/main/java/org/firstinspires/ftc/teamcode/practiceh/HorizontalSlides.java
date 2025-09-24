package org.firstinspires.ftc.teamcode.practiceh;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizontalSlides {

    private Servo rightSlidesServo;
    private Servo leftSlidesServo;

    public HorizontalSlides(HardwareMap hardwareMap) {
        rightSlidesServo = hardwareMap.get(Servo.class, "rightSlidesServo");
        leftSlidesServo = hardwareMap.get(Servo.class,"leftSlidesServo");
    }
}
