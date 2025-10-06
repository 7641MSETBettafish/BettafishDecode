package org.firstinspires.ftc.robotcontroller.practiceh;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {
    private Servo rightIntakeServo;
    private Servo leftIntakeServo;
    private DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
    }

    public class StartIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeMotor.setPower(1);
            leftIntakeServo.setPosition(1);
            rightIntakeServo.setPosition(1);

            return false;
        }

    }

    public class StopIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeMotor.setPower(0);
            leftIntakeServo.setPosition(0);
            rightIntakeServo.setPosition(0);

            return false;
        }

    }

}
