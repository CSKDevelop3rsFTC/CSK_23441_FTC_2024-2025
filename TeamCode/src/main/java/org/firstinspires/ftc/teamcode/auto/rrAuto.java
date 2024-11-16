package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;


// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "rrAuto", group = "Autonomous")
public class rrAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d beginPose = new Pose2d(-63, -11, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-41,0), Math.PI / 2) // move up to center rungs

                        .strafeTo(new Vector2d(-33, 0)) // go forward to line up specimen

                        .waitSeconds(2) // replace later with action of placing specimen

                        .strafeTo(new Vector2d(-45, 0)) // move backwards a bit

                        .strafeTo(new Vector2d(-47, 49)) // move to sample one

                        .turnTo(Math.toRadians(-180)) // do a 180 to have intake facing sample

                        .waitSeconds(2) // intake sample

                        .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position

                        .turnTo(Math.toRadians(130)) // turn around to the basket

                        .waitSeconds(2) // drop the pixels

                        .strafeTo(new Vector2d(-44.8, 61)) // move to sample position 2

                        .turnTo(Math.toRadians(-180))

                        .waitSeconds(2) // get the sample

                        .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position

                        .turnTo(Math.toRadians(130)) // turn around to the basket

                        .waitSeconds(2)

                        .turnTo(Math.toRadians(0))

                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(0,-24), Math.PI / 2) // move up to center rungs

                        //.setTangent(Math.PI / 2)
                        //.setReversed(true)
                        //.splineToConstantHeading(new Vector2d(0, 0), Math.PI / 2)

                        .build());

    }

}
