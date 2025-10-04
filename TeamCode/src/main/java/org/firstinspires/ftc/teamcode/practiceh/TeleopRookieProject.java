package org.firstinspires.ftc.teamcode.practiceh;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.*;


@TeleOp
public class TeleopRookieProject extends LinearOpMode {

    enum IntakeState {ON, OFF}
    enum SlidesState {ON, OFF}

    @Override
    public void runOpMode() {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse motors??
        // frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake intake = new Intake(hardwareMap);
        HorizontalSlides slides = new HorizontalSlides(hardwareMap);

        IntakeState intakeState = IntakeState.OFF;
        SlidesState slidesState = SlidesState.OFF;


        waitForStart();

        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            frontLeftMotor.setPower(y + x + rx);
            backLeftMotor.setPower(y - x + rx);
            frontRightMotor.setPower(y - x - rx);
            backRightMotor.setPower(y + x - rx);

            // Fixing ratios & normalizing motor powers
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            List<Action> runningActions = new ArrayList<>();

            switch (intakeState) {
                case OFF:
                    if(gamepad1.x) {
                        runningActions.add(new SequentialAction(
                                intake.new StartIntake()
                        ));
                        intakeState = IntakeState.ON;
                    }
                    break;

                case ON:
                    if(gamepad1.x) {
                        runningActions.add(new SequentialAction(
                                intake.new StopIntake()
                        ));
                        intakeState = IntakeState.OFF;
                    }
                    break;
            }

            switch (slidesState) {
                case OFF:
                    if(gamepad1.y) {
                        runningActions.add(new SequentialAction(
                                slides.new ExtendSlides()
                        ));
                        slidesState = SlidesState.ON;

                    }
                    break;

                case ON:
                    if(gamepad1.y) {
                        runningActions.add(new SequentialAction(
                                slides.new RetractSlides()
                        ));
                        slidesState = slidesState.OFF;
                    }
                    break;
            }

        }
    }

}
