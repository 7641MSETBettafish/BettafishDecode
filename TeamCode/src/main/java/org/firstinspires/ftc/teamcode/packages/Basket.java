package org.firstinspires.ftc.teamcode.packages;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Basket {


        private Servo basketServo;


        public Basket(HardwareMap hardwareMap) {
           basketServo = hardwareMap.get(Servo.class, "intakeServo");





    }

}
