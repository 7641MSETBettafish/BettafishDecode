package org.firstinspires.ftc.robotcontroller.practices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Basket1 {


        private Servo basketServo;


        public Basket1(HardwareMap hardwareMap) {
           basketServo = hardwareMap.get(Servo.class, "intakeServo");





    }

}
