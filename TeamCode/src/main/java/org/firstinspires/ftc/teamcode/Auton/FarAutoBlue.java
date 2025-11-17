package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;

import java.lang.Math;

@Config
@Autonomous(preselectTeleOp = "Teleop")
public class FarAutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-58, 58, Math.toRadians(-52));

        Shooter shooter = new Shooter(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // -------------------------------
        // PATHS BROKEN INTO SEGMENTS
        // -------------------------------

        // ---- SHOOT 1 ----
        Action toShoot1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-32, 20), Math.toRadians(-51))
                .build();

        // ---- INTAKE 1 ----
        Action toIntake1 = drive.actionBuilder(new Pose2d(-32, 20, Math.toRadians(-51)))
                .strafeToLinearHeading(new Vector2d(0, 28), Math.toRadians(90))
                .waitSeconds(0.01)
                .lineToY(68)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(0, 80), Math.toRadians(180))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-32, 20), Math.toRadians(-51))
                .build();

        // ---- INTAKE 2 ----
        Action toIntake2 = drive.actionBuilder(new Pose2d(-32, 20, Math.toRadians(-51)))
                .strafeToLinearHeading(
                        new Vector2d(25, 30),
                        Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 100)
                )
                .waitSeconds(0.01)
                .lineToY(72)
                .waitSeconds(0.01)
                .strafeToLinearHeading(
                        new Vector2d(-30, 15),
                        Math.toRadians(-53),
                        null,
                        new ProfileAccelConstraint(-60, 100)
                )
                .build();

        // ---- INTAKE 3 ----
        Action toIntake3 = drive.actionBuilder(new Pose2d(-32, 20, Math.toRadians(-53)))
                .strafeToLinearHeading(
                        new Vector2d(50, 30),
                        Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 100)
                )
                .waitSeconds(0.01)
                .lineToY(70)
                .waitSeconds(0.01)
                .strafeToLinearHeading(
                        new Vector2d(-32, 20),
                        Math.toRadians(-53),
                        null,
                        new ProfileAccelConstraint(-60, 100)
                )
                .build();

        // ---- PARK ----
        Action park = drive.actionBuilder(new Pose2d(-32, 20, Math.toRadians(-53)))
                .strafeToLinearHeading(
                        new Vector2d(-10, 40),
                        Math.toRadians(0),
                        null,
                        new ProfileAccelConstraint(-60, 100)
                )
                .build();


        waitForStart();

        // ==================================================
        // MASTER AUTON SEQUENCE (MATCHES AUTO BLUE)
        // ==================================================

        Actions.runBlocking(
                new ParallelAction(

                        shooter.run(3175),          // runs whole auton
                        intake.run(),               // runs whole auton
                        Context.updatePosition(drive, 0),

                        new SequentialAction(

                                // ---- SHOOT 1 ----
                                toShoot1,
                                transfer.farfullLoad(),

                                // ---- INTAKE 1 ----
                                new ParallelAction(
                                        toIntake1,
                                        transfer.run()
                                ),
                                transfer.farfullLoad(),

                                // ---- INTAKE 2 ----
                                new ParallelAction(
                                        toIntake2,
                                        transfer.run()
                                ),
                                transfer.farfullLoad(),

                                // ---- INTAKE 3 ----
                                new ParallelAction(
                                        toIntake3,
                                        transfer.run()
                                ),
                                transfer.farfullLoad(),

                                // ---- PARK ----
                                park
                        )
                )
        );
    }
}
