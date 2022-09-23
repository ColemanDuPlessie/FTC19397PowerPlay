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

import static org.firstinspires.ftc.teamcode.backend.cv.TeamShippingElementDetector.POSITIONS.LEFT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemPosition;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemSpeed;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.MakeIntakeVertical;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.RunRunnable;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.RunRunnableOnce;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.SpinDuck;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.WaitMillis;
import org.firstinspires.ftc.teamcode.backend.cv.TeamShippingElementDetector;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.backend.utilities.ToggleButton;
import org.openftc.easyopencv.OpenCvCameraException;


/**
 * I should probably document this...
 */

@Autonomous(name="Warehouse Side Auto (THIS ONE)")
@Config
public class WarehouseAutoWithSlides extends CommandbasedOpmode {

    SampleMecanumDrive drive;
    TrajectorySequence deliverPreload;
    TrajectorySequence enterWarehouse;
    TrajectorySequence enterWarehouseFinalTime;
    TrajectorySequence secondCycleExtraDepth;
    TrajectorySequence deliverFromWarehouse;
    TeamShippingElementDetector tagDetector;
    TeamShippingElementDetector.POSITIONS tagPosition = null;

    public double STARTX = 12;
    public double STARTY = 63;
    public double STARTTHETA = 90;
    public double DUMPX = 5;
    public double DUMPY = 32;
    public double DUMPTHETA = -150;
    public double PREBARRIERX = 12;
    public double PREBARRIERY = 69; // 70;
    public double PREBARRIERTHETA = 0;
    public double RELOCALIZATIONERRORX = -7; // -2.5; // DECREASING this number causes the robot to drift in the POSITIVE X direction
    public double RELOCALIZATIONERRORY = 0; // -1.25; // DECREASING this number causes the robot to drift in the POSITIVE Y direction
    public double BARRIERTRAVELDISTANCE = 38; // 34;
    public double SECONDCYCLETRAVELDISTANCE = 5;
    public double CYCLEDUMPX = 0;
    public double CYCLEDUMPY = 39;
    public double CYCLEDUMPTHETA = -150;

    public double DELIVERYRELOCALIZATIONDIST = RELOCALIZATIONERRORY;

    private double MAGICNUMBERMULT = 1;

    private int tagDetectionFails = 0;

    TrajectoryVelocityConstraint maxVelo = (a, pose, b, c) -> Math.max(21.0, Math.min(50.0, 50.0 - pose.getX()));

    double startHeading;

    @Override
    public void init() {

        if (!SetDrivingStyle.isBlue) {
            STARTY *= -1;
            STARTTHETA *= -1;
            DUMPY *= -1;
            DUMPTHETA *= -1;
            PREBARRIERY *= -1;
            PREBARRIERTHETA *= -1;
            RELOCALIZATIONERRORY *= -1;
            CYCLEDUMPY *= -1;
            CYCLEDUMPTHETA *= -1;
            DELIVERYRELOCALIZATIONDIST *= -1;
            MAGICNUMBERMULT *= -1;
        }

        robot.init(hardwareMap, false);

        startHeading = robot.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, Math.toRadians(STARTTHETA));

        tagDetector = new TeamShippingElementDetector(hardwareMap, 5.0, -3.0);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        deliverPreload = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(STARTX, DUMPY+24*MAGICNUMBERMULT), Math.toRadians(STARTTHETA + 180))
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
                .build();

        enterWarehouse = drive.trajectorySequenceBuilder(new Pose2d(DUMPX, DUMPY, DUMPTHETA))
                .setVelConstraint(maxVelo)
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 0.0, Subsystem.SLIDES, 0.0));
                    })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.0, Subsystem.ARM, 0.0));
                })
                .splineToLinearHeading(new Pose2d(PREBARRIERX, PREBARRIERY, Math.toRadians(PREBARRIERTHETA)), Math.toRadians(PREBARRIERTHETA))
                .splineToConstantHeading(new Vector2d(PREBARRIERX+BARRIERTRAVELDISTANCE, PREBARRIERY), Math.toRadians(PREBARRIERTHETA))
                .build();

        secondCycleExtraDepth = drive.trajectorySequenceBuilder(new Pose2d(PREBARRIERX+BARRIERTRAVELDISTANCE, PREBARRIERY, PREBARRIERTHETA))
                .setVelConstraint(maxVelo)
                .lineToConstantHeading(new Vector2d(PREBARRIERX+BARRIERTRAVELDISTANCE+SECONDCYCLETRAVELDISTANCE, PREBARRIERY))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(PREBARRIERX+BARRIERTRAVELDISTANCE, PREBARRIERY))
                .build();

        deliverFromWarehouse = drive.trajectorySequenceBuilder(new Pose2d(PREBARRIERX+BARRIERTRAVELDISTANCE+RELOCALIZATIONERRORX, PREBARRIERY+RELOCALIZATIONERRORY, PREBARRIERTHETA))
                .setVelConstraint(maxVelo)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    scheduler.clearSubsystem(Subsystem.INTAKE);
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.1, Subsystem.ARM, 0.1));
                })
                .splineToConstantHeading(new Vector2d(PREBARRIERX, PREBARRIERY+DELIVERYRELOCALIZATIONDIST), Math.toRadians(PREBARRIERTHETA+180))
                .splineToLinearHeading(new Pose2d(CYCLEDUMPX, CYCLEDUMPY, CYCLEDUMPTHETA), Math.toRadians(DUMPTHETA))
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
                .build();

        enterWarehouseFinalTime = drive.trajectorySequenceBuilder(new Pose2d(DUMPX, DUMPY, DUMPTHETA))
                .setVelConstraint((a, b, c, d) -> 60)
                .addTemporalMarker(() -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.arm, () -> 0.05, Subsystem.ARM, 0.0));
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    scheduler.scheduleCommand(new HoldSubsystemPosition(robot.slides, () -> 0.0, Subsystem.SLIDES, 0.0));
                })
                .splineToLinearHeading(new Pose2d(PREBARRIERX, PREBARRIERY, Math.toRadians(PREBARRIERTHETA)), Math.toRadians(PREBARRIERTHETA))
                .splineToConstantHeading(new Vector2d(PREBARRIERX+BARRIERTRAVELDISTANCE, PREBARRIERY), Math.toRadians(PREBARRIERTHETA))
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
        Command relocalize = new RunRunnableOnce(() -> drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(RELOCALIZATIONERRORX, RELOCALIZATIONERRORY, 0))));
        scheduler.scheduleCommand(new SequentialCommandGroup(
                new FollowRRTraj(drive, deliverPreload),
                new FollowRRTraj(drive, enterWarehouse),
                new WaitMillis(timer, 500),
                relocalize,
                new FollowRRTraj(drive, deliverFromWarehouse),
                new FollowRRTraj(drive, enterWarehouse),
                new FollowRRTraj(drive, secondCycleExtraDepth),
                relocalize,
                new FollowRRTraj(drive, deliverFromWarehouse),
                new FollowRRTraj(drive, enterWarehouseFinalTime)));
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