/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.commandbased.SubsystemInUseException;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.DriveFromGamepad;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemPosition;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemSpeed;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.MakeIntakeVertical;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.RunRunnableOnce;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.SetSubsystemSpeed;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.SpinDuck;
import org.firstinspires.ftc.teamcode.backend.cv.TeamShippingElementDetector;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.backend.utilities.RadioButtons;
import org.firstinspires.ftc.teamcode.backend.utilities.ToggleButton;
import org.openftc.easyopencv.OpenCvCameraException;

import java.util.HashMap;
import java.util.function.Supplier;


/**
 * I should probably document this...
 */

@Autonomous(name="Carousel Side Auto (THIS ONE)")
@Config
public class CarouselAutoWithSlides extends CommandbasedOpmode {

    SampleMecanumDrive drive;
    TrajectorySequence firstDump;
    TrajectorySequence carousel;
    TrajectorySequence secondDump;
    TeamShippingElementDetector tagDetector;
    TeamShippingElementDetector.POSITIONS tagPosition = null;

    public double STARTX = -36;
    public double STARTY = -63;
    public double STARTTHETA = -90;
    public double DUMPX = -29;
    public double DUMPY = -29;
    public double DUMPTHETA = 15;
    public double WALLBUMPX = -68;
    public double WALLBUMPY = -40;
    public double WALLBUMPTHETA = -90;
    public double WALLBUMPAPPROACHDIST = 9;
    public double WALLBUMPENDDIST = 3;
    public double WALLBUMPSPEED = 10;
    public double DUCKX = -59;
    public double DUCKY = -58;
    public double DUCKTHETA = -90;
    public double CONSUMEDUCKSTARTX = -56.5;
    public double CONSUMEDUCKY = -62.5;
    public double CONSUMEDUCKENDX = -30;
    public double CONSUMEDUCKSPEED = 10;
    public double RELOCALIZATIONERRORX = 0; // -3.0;
    public double RELOCALIZATIONERRORY = 8.0; // 5.5; // 8.0;
    public double PARKX = -60;
    public double PARKY = -36;
    public double PARKTHETA = 180;

    private double MAGICNUMBERMULTIPLIER = 1;

    private int tagDetectionFails = 0;

    double startHeading;

    @Override
    public void init() {

        if (SetDrivingStyle.isBlue) {
            STARTY *= -1;
            STARTTHETA *= -1;
            DUMPY *= -1;
            DUMPTHETA *= -1;
            WALLBUMPY *= -1;
            WALLBUMPTHETA *= -1;
            WALLBUMPAPPROACHDIST *= -1;
            WALLBUMPENDDIST *= -1;
            DUCKY *= -1;
            DUCKTHETA *= -1;
            DUCKTHETA += 90;
            DUCKX -= 6;
            DUCKY += 3;
            CONSUMEDUCKSTARTX -= 6;
            CONSUMEDUCKY *= -1;
            RELOCALIZATIONERRORY *= -1;
            RELOCALIZATIONERRORY += 6;
            RELOCALIZATIONERRORX += 2;
            PARKY *= -1;
            PARKTHETA *= -1;
            MAGICNUMBERMULTIPLIER *= -1;
        }

        robot.init(hardwareMap, false);

        startHeading = robot.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, Math.toRadians(STARTTHETA));

        tagDetector = new TeamShippingElementDetector(hardwareMap, 5.0, -3.0);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        firstDump = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(STARTX, DUMPY-24*MAGICNUMBERMULTIPLIER), Math.toRadians(STARTTHETA + 180))
                .splineTo(new Vector2d(DUMPX, DUMPY), Math.toRadians(DUMPTHETA))
                .addTemporalMarker(() -> {
                    switch ((tagPosition == null) ? TeamShippingElementDetector.POSITIONS.RIGHT : tagPosition) {
                        case LEFT:
                            scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 0.15, Subsystem.SLIDES, 0.0));
                            break;
                        case MIDDLE:
                            scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 0.5, Subsystem.SLIDES, 0.0));
                            break;
                        default:
                            scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 1.0, Subsystem.SLIDES, 0.0));
                            break;
                    }
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.3, Subsystem.ARM, 0.0));
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.65, Subsystem.ARM, 0.0));
                })
                .setReversed(false)
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 0.0, Subsystem.SLIDES, 0.0));
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.05, Subsystem.ARM, 0.0));
                })
                .splineToLinearHeading(new Pose2d(WALLBUMPX+WALLBUMPAPPROACHDIST, WALLBUMPY, Math.toRadians(WALLBUMPTHETA)), Math.toRadians(180*MAGICNUMBERMULTIPLIER))
                .setVelConstraint((a, b, c, d) -> WALLBUMPSPEED)
                .lineToConstantHeading(new Vector2d(WALLBUMPX, WALLBUMPY))
                .lineToConstantHeading(new Vector2d(WALLBUMPX+WALLBUMPENDDIST, WALLBUMPY))
                .build();

        carousel = drive.trajectorySequenceBuilder(new Pose2d(-66.0+WALLBUMPENDDIST, WALLBUMPY, Math.toRadians(WALLBUMPTHETA)))
                .splineToLinearHeading(new Pose2d(DUCKX, DUCKY, Math.toRadians(DUCKTHETA)), Math.toRadians(DUCKTHETA))
                .addTemporalMarker(() -> {scheduler.scheduleCommand(new SpinDuck(robot.carousel, SetDrivingStyle.isBlue, timer, true));})
                .waitSeconds(3.2)
                .setVelConstraint((a, b, c, d) -> CONSUMEDUCKSPEED)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    scheduler.scheduleCommand(new HoldSubsystemSpeed(robot.intake, () -> 0.5, Subsystem.INTAKE));
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.0, Subsystem.ARM, 0.0));

                })
                .splineToLinearHeading(new Pose2d(CONSUMEDUCKSTARTX, DUCKY, Math.toRadians(-90*MAGICNUMBERMULTIPLIER)), Math.toRadians(-90*MAGICNUMBERMULTIPLIER))
                .splineToConstantHeading(new Vector2d(CONSUMEDUCKSTARTX, CONSUMEDUCKY), Math.toRadians(-90*MAGICNUMBERMULTIPLIER))
                .lineToConstantHeading(new Vector2d(CONSUMEDUCKENDX, CONSUMEDUCKY))
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.05, Subsystem.ARM, 0.0));
                    scheduler.clearSubsystem(Subsystem.INTAKE);
                })
                .build();

        secondDump = drive.trajectorySequenceBuilder(new Pose2d(CONSUMEDUCKENDX+RELOCALIZATIONERRORX, CONSUMEDUCKY+RELOCALIZATIONERRORY, Math.toRadians(-90*MAGICNUMBERMULTIPLIER)))
                .setReversed(true)
                .splineTo(new Vector2d(DUMPX, DUMPY), Math.toRadians(DUMPTHETA))
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 1.0, Subsystem.SLIDES, 0.0));
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.3, Subsystem.ARM, 0.0));
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.65, Subsystem.ARM, 0.0));
                })
                .setReversed(false)
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.05, Subsystem.ARM, 0.0));
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 0.0, Subsystem.SLIDES, 0.0));
                })
                .lineToLinearHeading(new Pose2d(PARKX, PARKY, Math.toRadians(PARKTHETA)))
                .build();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        TeamShippingElementDetector.POSITIONS guess = tagDetector.getTagPosition();

        if (guess == null) {
            tagDetectionFails += 1;
        }
        if (tagDetectionFails > 20 || guess != null) {
            tagDetectionFails = 0;
            tagPosition = guess;
        }

        if (tagPosition != null) {
            telemetry.addLine("Tag is in the {} position".format(tagPosition.name()));
        } else { telemetry.addLine("Tag is not detected"); }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        try {
            tagDetector.close();
        } catch (OpenCvCameraException e) {
            telemetry.addLine("Camera was not initialized. CV pipeline replaced by default behavior.");
        }
        scheduler.setDefaultCommand(new MakeIntakeVertical(robot.intake, timer));
        Command relocalize = new RunRunnableOnce(() -> drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(RELOCALIZATIONERRORX, RELOCALIZATIONERRORY, 0))));
        Command relocalizeIntoField = new RunRunnableOnce(() -> drive.setPoseEstimate(new Pose2d(Math.max(-66.0, drive.getPoseEstimate().getX()), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())));
        scheduler.scheduleCommand(new SequentialCommandGroup(
                new FollowRRTraj(drive, firstDump),
                relocalizeIntoField,
                new FollowRRTraj(drive, carousel),
                relocalize,
                new FollowRRTraj(drive, secondDump)));
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        AutoToTeleopContainer.getInstance().setAngleDelta(startHeading-robot.getHeading()+Math.toRadians(180));
    }
}