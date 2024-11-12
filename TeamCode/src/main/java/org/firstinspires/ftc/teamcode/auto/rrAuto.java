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
        Pose2d beginPose = new Pose2d(-64, -11, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                        //.splineTo(new Vector2d(0, 60), Math.PI)
                        //.lineToX(24)
                        //.lineToX(0)
                        //.strafeTo(new Vector2d(24, 0))
                        //.strafeTo(new Vector2d(24, 24))
                        //.strafeTo(new Vector2d(0, 24))
                        //.strafeTo(new Vector2d(0, 0))

                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-41,0), Math.PI / 2)

                        .strafeTo(new Vector2d(-33, 0))

                        .waitSeconds(3)

                        .setTangent(Math.PI / 2)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(0, 0), Math.PI / 2)

                        //.setTangent(Math.PI / 2)
                        //.setReversed(true)
                        //.splineToConstantHeading(new Vector2d(0, 0), Math.PI / 2)

                        .build());

    }

}
