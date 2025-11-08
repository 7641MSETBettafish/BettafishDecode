package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

    public static boolean fieldCentric = false;
    public static boolean inverted = false;

    MecanumDrive drive;
    Intake intake;
    Transfer transfer;
    Shooter shooter;


    enum IntakeState {ON, OFF}
    enum ShooterState {ON, OFF}

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

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
        transfer = new Transfer(hardwareMap);

        PIDController headingPID = new PIDController(kPh, 0, 0);

//        tagProcessor = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(1421.04,1421.04,649.331,357.761)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
//                .setCameraResolution(new Size(1280, 720))
//                .build();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        double goalDistance = 0;

        boolean angleHold = false;

        ElapsedTime time = new ElapsedTime();

        IntakeState intakeState = IntakeState.OFF;
        ShooterState shooterState = ShooterState.OFF;

        Control shooterControl = new Control();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            time.reset();

            headingPID.setPID(kPh, 0, 0);

            Pose2d pose = drive.localizer.getPose();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            goalDistance = Math.sqrt(Math.pow(pose.position.x - goalPosition.position.x, 2) + Math.pow(pose.position.y - goalPosition.position.y, 2));


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (angleHold) {
                double dxGoal = goalPosition.position.x - pose.position.x;
                double dyGoal = goalPosition.position.y - pose.position.y;
                double targetAngle = Math.atan2(dyGoal, dxGoal);
                double headingError = targetAngle - pose.heading.toDouble();
                headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                telemetry.addData("target angle", Math.toDegrees(targetAngle));
                telemetry.addData("heading error", Math.toDegrees(headingError));

                rx = headingPID.calculate(headingError, targetAngle);

            }

            if (fieldCentric) {
                double botHeading = pose.heading.toDouble();

                x = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                y = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (inverted) {
                frontLeftPower = -frontLeftPower;
                backLeftPower = -backLeftPower;
                frontRightPower = -frontRightPower;
                backRightPower = -backRightPower;
            }

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

            boolean rightTriggerHeld = currentGamepad1.right_trigger > 0.6;

            switch (shooterState) {
                case OFF:
                    if (rightTriggerHeld) {
                        runningActions.add(new SequentialAction(
                                shooterControl.start(),
                                //shooter.powerUp(goalDistance),
                                transfer.load(),
                                shooterControl.end()
                        ));
                        shooterState = ShooterState.ON;
                    }
                    break;
                case ON:
                    if (shooterControl.isFinished()) {
                        runningActions.add(transfer.run());
                        shooterState = ShooterState.OFF;
                    }

                    break;
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                angleHold = !angleHold;
            }

            if (currentGamepad1.options && !previousGamepad1.options) {
                drive.localizer.setPose(new Pose2d(-48, -36, 0));
            }

            shooter.targetRPM = Shooter.calculateRPM(goalDistance);

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

            drive.updatePoseEstimate();

            telemetry.addData("robot X", drive.localizer.getPose().position.x);
            telemetry.addData("robot Y", drive.localizer.getPose().position.y);
            telemetry.addData("robot H", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("goal distance", goalDistance);

            telemetry.addData("flywheel RPM", shooter.RPM);
            telemetry.addData("target RPM", shooter.targetRPM);

            telemetry.addData("intake state", intakeState.toString());
            telemetry.addData("intake distance", intake.getDistance());

            telemetry.addData("right transfer distance", transfer.rightTransferSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("transfer ticks", transfer.transferMotor.getCurrentPosition());

            telemetry.addData("shooter state", shooterState.toString());
            telemetry.addData("shooter busy", shooterControl.isBusy());

            telemetry.addData("left stick" , currentGamepad1.left_stick_y);
            telemetry.addData("right stick" , currentGamepad1.right_stick_y);

            telemetry.addData("angle hold", angleHold);

            telemetry.addData("loop time", time.milliseconds());
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }

    }
}
