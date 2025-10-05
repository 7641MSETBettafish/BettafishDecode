package org.firstinspires.ftc.teamcode.packages;



import android.transition.Slide;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.practiceh.HorizontalSlides;
import org.firstinspires.ftc.teamcode.practiceh.Intake;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class TeleOpProject extends LinearOpMode {

    enum IntakeTransfer {ON, OFF}
    enum SlideDeposit {ON , OFF}




    @Override
    public void runOpMode() {

      IntakeTransfer intakeTransfer = IntakeTransfer.OFF;
      SlideDeposit slidesDeposit = SlideDeposit.OFF;

        waitForStart();

        while (opModeIsActive()) {


            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; //this is to compentsate for imperfect strafing
            double rx = gamepad1.right_stick_x; // the 1.1 gives 10 percent boost to make it more perfect

            frontLeftMotor.setPower(y + x + rx); //x =strafe y=forward/backward  rx= rotate
            backLeftMotor.setPower(y - x + rx); //assigns diff roles to each motor
            frontRightMotor.setPower(y - x - rx);
            backRightMotor.setPower(y + x - rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx / denominator);


            Intake intake = new Intake(hardwareMap);
            HorizontalSlides slides = new HorizontalSlides(hardwareMap);

            List<Action> runningActions = new ArrayList<>();

            switch (intakeTransfer) {
                case OFF:
                    if (gamepad1.x) {
                        runningActions.add(new SequentialAction(
                                intake.new StartIntake()
                        ));

                        intakeTransfer = IntakeTransfer.ON;


                    }

                case ON:
                    if (gamepad1.x) {
                        runningActions.add(new SequentialAction(
                                intake.new StopIntake()
                        ));

                        intakeTransfer = IntakeTransfer.OFF;

                    }
            }
            switch (slidesDeposit) {

                case OFF:
                    if (gamepad1.y) {
                        runningActions.add(new SequentialAction(
                                slides.new ExtendSlides()
                        ));
                        slidesDeposit = SlideDeposit.ON;

                    }

                case ON:
                    if (gamepad1.y) {
                        runningActions.add(new SequentialAction(
                                slides.new RetractSlides()
                        ));
                        slidesDeposit = SlideDeposit.OFF;

                    }


            }
        }
        }
    }

