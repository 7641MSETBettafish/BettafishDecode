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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, -50, Math.toRadians(54)))
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39))
                .waitSeconds(1.5)
                .splineToSplineHeading(new Pose2d(-16.5, -20, Math.toRadians(-90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-17.5,-55), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20, -20), Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(-4, -55), Math.toRadians(-75))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39))
                .waitSeconds(0.8)
                .splineToSplineHeading(new Pose2d(1.5, -24, Math.toRadians(-90)), Math.toRadians(-75))
                .lineToY(-62)
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(-30, -25, Math.toRadians(39)), Math.toRadians(-30))
                .waitSeconds(1.6)
                .splineToSplineHeading(new Pose2d(23, -24, Math.toRadians(-90)), Math.toRadians(-75))
                .lineToY(-60)
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(-30, -25, Math.toRadians(39)), Math.toRadians(-75))
                .waitSeconds(1)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}