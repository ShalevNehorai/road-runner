package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Pose2d(30, 50, Math.toRadians(-10)))
                        .splineTo(new Pose2d(60, 40, Math.toRadians(-70)))
                .build()
        );

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .lineTo(new Vector2d(60, 100))
                .splineTo(new Pose2d(60, 150, Math.toRadians(180)))
                .build()
        );

    }
}
