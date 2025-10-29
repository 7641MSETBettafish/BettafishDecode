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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Control;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends LinearOpMode {

    public static double startX = 0;
    public static double startY = 0;
    public static double startH = 0;

    MecanumDrive drive;
    Intake intake;
    Transfer transfer;
    Shooter shooter;

    final Pose2d goalPosition = new Pose2d(-60, 63,0);
    enum IntakeState {ON, OFF}
    enum ShooterState {ON, OFF}

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startH));

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Gamepad previousGamepad1 = new Gamepad();
        //Gamepad previousGamepad2 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        //Gamepad currentGamepad2 = new Gamepad();

        double goalDistance = 0;
        double balls = 0;
        boolean lastSense = false;

        ElapsedTime time = new ElapsedTime();

        IntakeState intakeState = IntakeState.OFF;
        ShooterState shooterState = ShooterState.OFF;

        Control shooterControl = new Control();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            time.reset();

            previousGamepad1.copy(currentGamepad1);
            //previousGamepad2 = currentGamepad2;
            currentGamepad1.copy(gamepad1);
            //currentGamepad2 = gamepad2;

            goalDistance = Math.sqrt(Math.pow(drive.localizer.getPose().position.x - goalPosition.position.x, 2) + Math.pow(drive.localizer.getPose().position.y - goalPosition.position.y, 2));


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double LFPower = (y + x + rx) / denominator;
            double LBPower = (y - x + rx) / denominator;
            double RFPower = (y - x - rx) / denominator;
            double RBPower = (y + x - rx) / denominator;

            drive.leftFront.setPower(LFPower);
            drive.leftBack.setPower(LBPower);
            drive.rightFront.setPower(RFPower);
            drive.rightBack.setPower(RBPower);

            boolean leftTriggerPressed = currentGamepad1.left_trigger > 0.9 && previousGamepad1.left_trigger < 0.9;

            switch (intakeState) {
                case OFF:
                    if (leftTriggerPressed) {
                        intake.intakeMotor.setPower(Intake.intakePower);
                        //transfer.transferMotor.setPower(Transfer.transferPower);
                        intakeState = IntakeState.ON;
                    }
                    break;
                case ON:
                    if (intake.ballSensed() && !lastSense) {
                        if (balls < 1) {
                            runningActions.add(transfer.run());
                        }
                        balls++;
                    }
                    if (balls >= 3 || leftTriggerPressed) {
                        intake.intakeMotor.setPower(0);
                        intakeState = IntakeState.OFF;
                    }
                    break;
            }

            lastSense = intake.ballSensed();

            boolean rightTriggerHeld = currentGamepad1.right_trigger > 0.9;

            switch (shooterState) {
                case OFF:
                    if (rightTriggerHeld && balls > 0) {
                        Action shotType = shooter.powerUp(goalDistance);
                        if (Math.abs(goalDistance - Shooter.Close) < 3) {
                            shotType = shooter.powerUp(Shooter.Distances.CLOSE);
                        } else if (Math.abs(goalDistance - Shooter.Middle) < 3) {
                            shotType = shooter.powerUp(Shooter.Distances.CLOSE);
                        } else if (Math.abs(goalDistance - Shooter.Far) < 3) {
                            shotType = shooter.powerUp(Shooter.Distances.CLOSE);
                        }

                        runningActions.add(new SequentialAction(
                                shooterControl.start(),
                                shotType,
                                transfer.load(),
                                shooterControl.end()
                        ));
                        shooterState = ShooterState.ON;
                    }
                    break;
                case ON:
                    if (shooterControl.isFinished()) {
                        balls--;
                        if (balls == 0) shooter.setPower(0);
                        shooterState = ShooterState.OFF;
                    }
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                drive.localizer.setPose(new Pose2d(-48, -36, 0));
            }

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
            telemetry.addData("robot H", drive.localizer.getPose().heading.toDouble());
            telemetry.addData("goal distance", goalDistance);
            telemetry.addData("balls", balls);
            telemetry.addData("left flywheel RPM", shooter.leftRPM);
            telemetry.addData("right flywheel RPM", shooter.rightRPM);
            telemetry.addData("intake state", intakeState.toString());
            telemetry.addData("intake distance", intake.getDistance());
            telemetry.addData("left transfer distance", transfer.leftTransferSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("right transfer distance", transfer.rightTransferSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("shooter state", shooterState.toString());
            telemetry.addData("shooter busy", shooterControl.isBusy());
            telemetry.addData("loop time", time.milliseconds());
            telemetry.addData("intake balls sensed", intake.ballSensed());
            telemetry.addData("last sensed", lastSense);
//            telemetry.addData("left trigger" , currentGamepad1.left_trigger);
//            telemetry.addData("right trigger" , currentGamepad1.right_trigger);
//            telemetry.addData("left on", leftTriggerPressed);
//            telemetry.addData("previous left trigger", previousGamepad1.left_trigger);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }

    }
}
