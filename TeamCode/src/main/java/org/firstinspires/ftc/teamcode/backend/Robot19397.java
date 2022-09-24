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

package org.firstinspires.ftc.teamcode.backend;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

/**
 * I should probably write this documentation...
 */
public class Robot19397
{
    /* Public OpMode members. */
    protected DcMotor  leftFront      = null;
    protected DcMotor  rightFront     = null;
    protected DcMotor  leftRear       = null;
    protected DcMotor  rightRear      = null;

    protected BNO055IMU imu = null;
    protected double startHeading;

    public ArmSubsystem arm;
    public SlidesSubsystem slides;

    private double leftFrontSpeed;
    private double rightFrontSpeed;
    private double leftRearSpeed;
    private double rightRearSpeed;


    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime timer;

    /* Constructor */
    public Robot19397(ElapsedTime timer){
        this.timer = timer;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        arm = new ArmSubsystem(timer, hwMap);
        slides = new SlidesSubsystem(timer, hwMap);

        // Define and Initialize Motors
        imu = hwMap.get(BNO055IMU.class, "chubimu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        leftFront  = hwMap.get(DcMotor.class, "LeftFront");
        rightFront = hwMap.get(DcMotor.class, "RightFront");
        leftRear  = hwMap.get(DcMotor.class, "LeftRear");
        rightRear = hwMap.get(DcMotor.class, "RightRear");
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        startHeading = getHeading();

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init(HardwareMap ahwMap, boolean isTeleop) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        arm = new ArmSubsystem(timer, hwMap);
        slides = new SlidesSubsystem(timer, hwMap, isTeleop);

        // Define and Initialize Motors
        imu = hwMap.get(BNO055IMU.class, "chubimu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        leftFront  = hwMap.get(DcMotor.class, "LeftFront");
        rightFront = hwMap.get(DcMotor.class, "RightFront");
        leftRear  = hwMap.get(DcMotor.class, "LeftRear");
        rightRear = hwMap.get(DcMotor.class, "RightRear");
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        if (isTeleop) {
            Double position = AutoToTeleopContainer.getInstance().getAngleDelta();
            if (position == null) {
                startHeading = getHeading();
            } else { startHeading = getHeading() + position;}
        }

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    private void setLeftFrontPower (double power) {
        if (leftFrontSpeed != power) {
            leftFrontSpeed = power;
            leftFront.setPower(power);
        }
    }
    private void setRightFrontPower(double power) {
        if (rightFrontSpeed != power) {
            rightFrontSpeed = power;
            rightFront.setPower(power);
        }
    }
    private void setLeftRearPower  (double power) {
        if (leftRearSpeed != power) {
            leftRearSpeed = power;
            leftRear.setPower(power);
        }
    }
    private void setRightRearPower (double power) {
        if (rightRearSpeed != power) {
            rightRearSpeed = power;
            rightRear.setPower(power);
        }
    }

    public void driveSimple(double forward, double turn, double strafe, double speed) {
        double leftFrontPower  = forward + turn + strafe;
        double rightFrontPower = forward - turn - strafe;
        double leftRearPower   = forward + turn - strafe;
        double rightRearPower  = forward - turn + strafe;
        driveNormalizedAngular(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower, speed);
    }

    public void driveFieldCentric(double yaxis, double turn, double xaxis, double speed) {
        double angle = startHeading - getHeading();
        double forward = Math.cos(angle) * yaxis + Math.sin(angle) * xaxis;
        double strafe = -Math.sin(angle) * yaxis + Math.cos(angle) * xaxis;
        driveSimple(forward, turn, strafe, speed);
    }

    public void driveNormalizedAngular(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower, double maxPower) {
        double maxSpeed = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));
        if (maxSpeed > 1) {
            leftFrontPower  /= maxSpeed;
            rightFrontPower /= maxSpeed;
            leftRearPower   /= maxSpeed;
            rightRearPower  /= maxSpeed;
        }
        double forward = (leftFrontPower + rightFrontPower + leftRearPower + rightRearPower) / 4;
        double strafe = (leftFrontPower - rightFrontPower - leftRearPower + rightRearPower) / 4;
        double turnAbs = Math.abs(leftFrontPower - rightFrontPower + leftRearPower - rightRearPower) / 4;
        double polar = Math.sqrt(forward * forward + strafe * strafe);
        if (polar > maxPower) {
            leftFrontPower /= polar / maxPower;
            rightFrontPower /= polar / maxPower;
            leftRearPower /= polar / maxPower;
            rightRearPower /= polar / maxPower;
        }
        if (Math.abs(turnAbs) > maxPower) {
            leftFrontPower  /= turnAbs / maxPower;
            rightFrontPower /= turnAbs / maxPower;
            leftRearPower   /= turnAbs / maxPower;
            rightRearPower  /= turnAbs / maxPower;
        }
        setLeftFrontPower(leftFrontPower);
        setRightFrontPower(rightFrontPower);
        setLeftRearPower(leftRearPower);
        setRightRearPower(rightRearPower);
    }

    public void update() {
        arm.update();
        slides.update();
    }
 }

