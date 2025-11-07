package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends LinearOpMode {

    public static double startX = 72 - 8.5;
    public static double startY = 15.25 / 2;
    public static double startH = 0;
    // <100 is blue
    // >=100 is red
    public static double goalSide = 0;

    public static double kPh = 0.1;
    public static double hDeadZone = 15;

    MecanumDrive drive;
    Intake intake;
    Transfer transfer;
    Shooter shooter;


    enum IntakeState {ON, OFF}
    enum ShooterState {ON, OFF}

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        Pose2d goalPosition;
        if (goalSide < 100) {
            goalPosition = new Pose2d(-60, -63,0);
        } else {
            goalPosition = new Pose2d(-60, 63,0);
        }


        drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startH));

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap) ;


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        double goalDistance = 0;
        double balls = 0;
        boolean lastSense = false;

        boolean angleHold = false;

        ElapsedTime time = new ElapsedTime();

        IntakeState intakeState = IntakeState.OFF;
        ShooterState shooterState = ShooterState.OFF;

        Control shooterControl = new Control();

        boolean shooterOn = false;
        boolean load = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            time.reset();

            Pose2d pose = drive.localizer.getPose();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            goalDistance = Math.sqrt(Math.pow(pose.position.x - goalPosition.position.x, 2) + Math.pow(pose.position.y - goalPosition.position.y, 2));


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = drive.localizer.getPose().heading.toDouble();

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            drive.leftFront.setPower(frontLeftPower);
            drive.leftBack.setPower(backLeftPower);
            drive.rightFront.setPower(frontRightPower);
            drive.rightBack.setPower(backRightPower);

            boolean leftTriggerPressed = currentGamepad1.left_trigger > 0.6 && previousGamepad1.left_trigger < 0.6;

            switch (intakeState) {
                case OFF:
                    if (leftTriggerPressed) {
                        intake.intakeMotor.setPower(Intake.intakePower);
                        runningActions.add(transfer.run());
                        //transfer.transferMotor.setPower(Transfer.transferPower);
                        intakeState = IntakeState.ON;
                    }
                    break;
                case ON:
                    if (leftTriggerPressed) {
                        intake.intakeMotor.setPower(0);
                        intakeState = IntakeState.OFF;
                    }
                    break;
            }

            lastSense = intake.ballSensed();

            boolean rightTriggerHeld = currentGamepad1.right_trigger > 0.6;

            switch (shooterState) {
                case OFF:
                    if (rightTriggerHeld /*&& balls > 0*/) {
                        runningActions.add(new SequentialAction(
                                shooterControl.start(),
                                shooter.powerUp(goalDistance),
                                transfer.load(),
                                shooterControl.end()
                        ));
                        //shooterOn = true;
                        shooterState = ShooterState.ON;
                    }
                    break;
                case ON:
                    if (shooterControl.isFinished()) {
                        //balls--;
                        //if (balls == 0) shooter.setPower(0);
                        runningActions.add(transfer.run());
                        shooterState = ShooterState.OFF;
                    }
//                    if (currentGamepad1.y && !previousGamepad1.y) {
//                        load = true;
//                        transfer.transferMotor.setPower(Transfer.transferPower);
//                        transfer.transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        transfer.transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                    if (currentGamepad1.a && !previousGamepad1.a) {
//                        shooterOn = false;
//                        shooterState = ShooterState.OFF;
//                    }
//
//                    if (load) {
//                        if (transfer.transferMotor.getCurrentPosition() >= Transfer.loadDistance) {
//                            transfer.transferMotor.setPower(0);
//                            load = false;
//                        }
//                    }
                    break;
            }
//            if (shooterOn) {
//                shooter.updatePID();
//            } else {
//                shooter.setPower(0);
//            }



            if (currentGamepad1.a && !previousGamepad1.a) {
                angleHold = !angleHold;
            }



            if (currentGamepad1.options && !previousGamepad1.options) {
                drive.localizer.setPose(new Pose2d(-48, -36, 0));
            }

            shooter.updateRPM();
            shooter.updatePID();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            shooter.targetRPM = 3500;
            drive.updatePoseEstimate();

            telemetry.addData("robot X", drive.localizer.getPose().position.x);
            telemetry.addData("robot Y", drive.localizer.getPose().position.y);
            telemetry.addData("robot H", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("goal distance", goalDistance);
            telemetry.addData("balls", balls);
            telemetry.addData("flywheel RPM", shooter.RPM);
            telemetry.addData("target RPM", shooter.targetRPM);

            telemetry.addData("intake state", intakeState.toString());
            telemetry.addData("intake distance", intake.getDistance());
            //telemetry.addData("left transfer distance", transfer.leftTransferSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("right transfer distance", transfer.rightTransferSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("shooter state", shooterState.toString());
            telemetry.addData("shooter busy", shooterControl.isBusy());
            telemetry.addData("loop time", time.milliseconds());
            telemetry.addData("intake balls sensed", intake.ballSensed());
            telemetry.addData("last sensed", lastSense);
            telemetry.addData("left stick" , currentGamepad1.left_stick_y);
            telemetry.addData("right stick" , currentGamepad1.right_stick_y);

            telemetry.addData("transfer ticks", transfer.transferMotor.getCurrentPosition());
            telemetry.addData("angle hold", angleHold);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }

    }
}
