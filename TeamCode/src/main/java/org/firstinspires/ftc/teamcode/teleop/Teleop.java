package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
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

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    double balls = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startH));

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        double goalDistance = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            previousGamepad1 = currentGamepad1;
            previousGamepad2 = currentGamepad2;
            currentGamepad1 = gamepad1;
            currentGamepad2 = gamepad2;

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

            if (currentGamepad1.left_trigger > 0.9 && previousGamepad1.left_trigger < 0.9) {
                if (balls < 3) {
                    runningActions.add(intake.run());
                    balls++;
                }
            }

            if (currentGamepad1.right_trigger > 0.9 && previousGamepad1.right_trigger < 0.9) {
                if (balls > 0) {
                    runningActions.add(new SequentialAction(
                            shooter.powerUp(goalDistance),
                            transfer.load()
                    ));
                }
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

            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }

    }
}
