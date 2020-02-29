package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.drive.tank.SampleTankDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(70,30, Math.toRadians(-30)), new ConstantInterpolator(-30))
                        .forward(20)
                        .build()
        );
        //drive.turnSync(Math.toRadians(-100));
        sleep(500);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .back(20)
                .splineTo(new Pose2d(70, 150, Math.toRadians(180)), new ConstantInterpolator(Math.toRadians(180)))
                .build()
        );
    }
}
