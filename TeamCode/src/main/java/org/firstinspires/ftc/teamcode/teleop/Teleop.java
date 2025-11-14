package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import androidx.annotation.NonNull;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends LinearOpMode {

    public static double startX = 0;
    public static double startY = 0
            ;
    public static double startH = 0;
    // <100 is blue
    // >=100 is red
    public static double goalSide = 1000;

    public static double kPh = 0.023;

    public static boolean fieldCentric = true;

    MecanumDrive drive;
    Intake intake;
    Transfer transfer;
    Shooter shooter;


    LynxModule myRevHub;
    double totalCurrent;



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
        myRevHub = hardwareMap.get(LynxModule.class, "Control Hub");

        totalCurrent = myRevHub.getCurrent(CurrentUnit.AMPS);
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
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .build();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        double goalDistance = 0;

        boolean angleHold = false;
        boolean shooterOn = false;

        int shootRPM = 0;

        ElapsedTime time = new ElapsedTime();

        IntakeState intakeState = IntakeState.OFF;
        ShooterState shooterState = ShooterState.OFF;

        Control shooterControl = new Control();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {


            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            Pose2d pose = drive.localizer.getPose();
            TelemetryPacket packet = new TelemetryPacket();


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
//
//            goalDistance = Math.hypot(pose.position.x - goalPosition.position.x,
//                    pose.position.y - goalPosition.position.y);

            double y = -currentGamepad1.left_stick_y;
            double x = currentGamepad1.left_stick_x;
            double rx = currentGamepad1.right_stick_x;

            if (fieldCentric) {
                double botHeading = pose.heading.toDouble() +
                        (goalSide < 100 ? Math.PI / 2 : -Math.PI / 2);
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                drive.leftFront.setPower((rotY + rotX + rx) / denominator);
                drive.leftBack.setPower((rotY - rotX + rx) / denominator);
                drive.rightFront.setPower((rotY - rotX - rx) / denominator);
                drive.rightBack.setPower((rotY + rotX - rx) / denominator);
            } else {
                x *= 1.1;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                drive.leftFront.setPower((y + x + rx) / denominator);
                drive.leftBack.setPower((y - x + rx) / denominator);
                drive.rightFront.setPower((y - x - rx) / denominator);
                drive.rightBack.setPower((y + x - rx) / denominator);
            }


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


            if (currentGamepad1.a && !previousGamepad1.a) angleHold = !angleHold;


            if (currentGamepad1.dpad_down) {
                transfer.transferMotor.setPower(-1);
                intake.intakeMotor.setPower(-1);
            }
            if (currentGamepad1.options && !previousGamepad1.options) {
                if (goalSide >= 100) {
                    drive.localizer.setPose(new Pose2d(54 + 17.0625, -24.625 - 7.75, 0));
                } else {
                    drive.localizer.setPose(new Pose2d(54 + 17.0625, 24.625 + 7.75, 0));
                }
            }
            if (currentGamepad2.x && !previousGamepad2.x) shootRPM = 0;
            if (currentGamepad2.a && !previousGamepad2.a) shootRPM = 3;
            if (currentGamepad2.b && !previousGamepad2.b) shootRPM = 4;


            switch (shootRPM) {
                case 0:
                    shooter.targetRPM = 0;
                    break;
                case 1:
                    shooter.targetRPM = Shooter.calculateRPM(goalDistance);
                    break;
                case 2:
                    shooter.targetRPM = 3000;
                    break;
                case 3:
                    shooter.targetRPM = 3300;
                    break;
                case 4:
                    shooter.targetRPM = 3650;
                    break;
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

            drive.updatePoseEstimate();

            telemetry.addData("robot X", drive.localizer.getPose().position.x);
            telemetry.addData("robot Y", drive.localizer.getPose().position.y);
            telemetry.addData("robot H", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("flywheel RPM", shooter.RPM);
            telemetry.addData("target RPM", shooter.targetRPM);
            telemetry.addData("intake state", intakeState.toString());
            telemetry.addData("right transfer distance", transfer.rightTransferSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("transfer ticks", transfer.transferMotor.getCurrentPosition());
            telemetry.addData("shooter state", shooterState.toString());
            telemetry.addData("shooter busy", shooterControl.isBusy());
            telemetry.addData("loop time", time.milliseconds());
            time.reset();

            telemetry.update();


        }

    }
}
