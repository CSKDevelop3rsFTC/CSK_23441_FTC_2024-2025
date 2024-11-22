package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.PI, Math.PI, 11.456382173167453)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-64, -11, 0))
/**
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-41,0), Math.PI / 2) // move up to center rungs

                .strafeTo(new Vector2d(-33, 0)) // go forward to line up specimen

                .waitSeconds(3) // replace later with action of placing specimen

                .strafeTo(new Vector2d(-45, 0)) // move backwards a bit

                .strafeTo(new Vector2d(-47, 49)) // move to sample one

                .turnTo(Math.toRadians(-180)) // do a 180 to have intake facing sample

                .waitSeconds(2) // intake sample

                .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position

                .turnTo(Math.toRadians(30)) // turn around to the basket

                .waitSeconds(3) // drop the pixels

                .strafeTo(new Vector2d(-44.8, 59)) // move to sample position 2

                .turnTo(Math.toRadians(-180))

                .waitSeconds(3) // get the sample

                .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position

                .turnTo(Math.toRadians(30)) // turn around to the basket
**/
                .setTangent(0)
                .strafeToConstantHeading(new Vector2d(-62,52))
                .turnTo(Math.toRadians(130))// move up to center rungs

                .strafeTo(new Vector2d(-12, 33)) // move to sample one

                .turnTo(Math.toRadians(30))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}