package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;


@Config
@Autonomous(preselectTeleOp = "Teleop")
public class Autoblue extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-52, -50, Math.toRadians(54));

        Shooter shooter = new Shooter(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Context context = new Context();
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // -------------------------------
        // PATHS BROKEN INTO SEGMENTS
        // -------------------------------

        // 1: GO TO FIRST SHOOTING POSITION
        Action toShoot1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-27, -20), Math.toRadians(45))
                .build();

        // 2: GO TO FIRST INTAKE POSITION
        Action toIntake1 = drive.actionBuilder(new Pose2d(-25, -20, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-35, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-41)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-25, -50), Math.toRadians(170))
                .waitSeconds(0.01)
                .splineToLinearHeading(new Pose2d(-27, -20, Math.toRadians(45)), Math.toRadians(45))
                .build();

        // 4: GO TO SECOND INTAKE
        Action toIntake2 = drive.actionBuilder(new Pose2d(-27, -20, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-10, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-51)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-27, -20), Math.toRadians(45))
                .build();


        // 6: THIRD INTAKE
        Action toIntake3 = drive.actionBuilder(new Pose2d(-27, -20, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(10, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-53)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-27, -20), Math.toRadians(45))
                .build();

        // 7: FINAL RETURN TO SHOOT

        // 8: PARK
        Action park = drive.actionBuilder(new Pose2d(-27, -20, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(0))
                .build();

        waitForStart();

        // ==================================================
        // MASTER AUTON SEQUENCE
        // ==================================================

        Actions.runBlocking(
                new ParallelAction(

                        // SHOOTER + INTAKE RUN FOR ENTIRE AUTON
                        shooter.run(3120),
                        intake.run(),
                        Context.updatePosition(drive, 0),

                        // MAIN PATH SEQUENCE
                        new SequentialAction(

                                // ---- SHOOT 1 ----
                                toShoot1,
                                transfer.fullLoad(),

                                // ---- INTAKE 1 (TRANSFER RUNS WHILE MOVING) ----
                                new ParallelAction(
                                        toIntake1,
                                        transfer.run()
                                ),

                                transfer.fullLoad(),

                                // ---- INTAKE 2 ----
                                new ParallelAction(
                                        toIntake2,
                                        transfer.run()
                                ),


                                transfer.fullLoad(),

                                // ---- INTAKE 3 ----
                                new ParallelAction(
                                        toIntake3,
                                        new RaceAction(
                                                transfer.run(),
                                                new SleepAction(4.5)
                                        )
                                ),


                                transfer.fullLoad(),

                                // ---- PARK ----
                                park
                        )
                )
        );
    }
}
