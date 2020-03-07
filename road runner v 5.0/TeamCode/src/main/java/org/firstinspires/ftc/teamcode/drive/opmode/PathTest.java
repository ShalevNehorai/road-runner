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
            .splineToLinearHeading(new Pose2d(70,60), Math.toRadians(-50))
            .build()
        );

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Vector2d(60, 70), Math.toRadians(-90))
                .build()
        );

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(80, 170), Math.toRadians(-90))
                .build()
        );

    }
}
