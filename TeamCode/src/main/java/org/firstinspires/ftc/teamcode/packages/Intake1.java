package org.firstinspires.ftc.teamcode.packages;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake1 {
    private Servo intakeServo;
    private Servo rotateServo;

    private DcMotor intakeMotor;

    public Intake1(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
       rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }




}
