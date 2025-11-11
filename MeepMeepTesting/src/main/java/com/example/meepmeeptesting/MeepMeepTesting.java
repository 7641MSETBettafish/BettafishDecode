package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 50, Math.toRadians(180), Math.toRadians(180), 16.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -54, Math.toRadians(54)))
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(0.85)
                .strafeToLinearHeading(new Vector2d(-37, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-45)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-25, -48), Math.toRadians(-150))
                .waitSeconds(0.01)
                .splineToLinearHeading(new Pose2d(-30, -20, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(-9, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-51)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(10, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-53)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}