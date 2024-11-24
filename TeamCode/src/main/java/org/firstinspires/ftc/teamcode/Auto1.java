package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class Auto1 extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-39.56, -62.50, Math.toRadians(90.00)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, 30, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .splineTo(new Vector2d(-47.83, -47.83), Math.toRadians(45.00))
                .lineToConstantHeading(new Vector2d(-60.80, -58.84))
                .build();

        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajectory0);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
