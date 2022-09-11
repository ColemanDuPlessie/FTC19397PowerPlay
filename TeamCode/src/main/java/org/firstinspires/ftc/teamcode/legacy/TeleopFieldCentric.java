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

package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.Robot19397;


/**
 * I should probably document this...
 */
/*
@Disabled
@TeleOp(name="Field-Centric Arcade Drive")
public class TeleopFieldCentric extends OpMode {

    Robot19397 robot = new Robot19397();
    final double defaultSpeed   = 0.75;
    final double topSpeed       = 1.0;
    final double minSpeed       = 0.167;
    boolean isFrontReversed     = false;
    boolean aAlreadyDown        = false;
    boolean bAlreadyDown        = false;

    enum BUTTONSTYLE {
        TOGGLE,
        HOLD
    }

    BUTTONSTYLE buttonstyle = BUTTONSTYLE.HOLD;

    @Override
    public void init() {

        robot.init(hardwareMap);

        telemetry.addData("Front Joystick Reversed? (toggle with a)", isFrontReversed);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        if (gamepad1.a != aAlreadyDown) {
            aAlreadyDown = gamepad1.a;
            if (aAlreadyDown) {
                isFrontReversed = !isFrontReversed;
            }
        }
        if (gamepad1.b != bAlreadyDown) {
            bAlreadyDown = gamepad1.b;
            if (bAlreadyDown) {
                switch (buttonstyle) {
                    case TOGGLE: buttonstyle = BUTTONSTYLE.HOLD; break;
                    case HOLD: buttonstyle = BUTTONSTYLE.TOGGLE; break;
                }
            }
        }
        telemetry.addData("Front Joystick Reversed? (toggle with a)", isFrontReversed);
        telemetry.addData("Button Style: (toggle with b)", buttonstyle == BUTTONSTYLE.HOLD ? "Hold" : "Toggle");
    }

    /*
     * Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {
        double forward = isFrontReversed ? gamepad1.left_stick_y : -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        double speed = defaultSpeed + gamepad1.right_trigger * (topSpeed-defaultSpeed) - gamepad1.left_trigger * (defaultSpeed-minSpeed);
        /*
        if      (gamepad1.a)                      {robot.setDuckServoPower(0.65);}
        else if (gamepad1.b)                      {robot.setDuckServoPower(-0.65);}
        else if (buttonstyle == BUTTONSTYLE.HOLD) {robot.setDuckServoPower(0);}
        /*
        if (gamepad1.right_bumper) {
            robot.intake.setSpeed(IntakeSubsystem.SPEED.ON);
        } else if (gamepad1.left_bumper) {
            robot.intake.setSpeed(IntakeSubsystem.SPEED.REVERSE);
        } else if (buttonstyle == BUTTONSTYLE.HOLD){
            robot.intake.setSpeed(IntakeSubsystem.SPEED.OFF);
        }

        if (buttonstyle == BUTTONSTYLE.TOGGLE && gamepad1.x) {
            robot.setDuckServoPower(0);
            robot.intake.setSpeed(IntakeSubsystem.SPEED.OFF);
        }


        if (gamepad1.left_stick_button) {
            forward = 0.0;
            strafe = 0.0;
            int armSetpoint = (int)((-gamepad1.left_stick_y + 1.0) * 110);
            robot.arm.setSetpoint(armSetpoint);
            robot.arm.goToPosition(ArmSubsystem.POSITION.CUSTOM);
        } else if (gamepad1.dpad_down) {robot.arm.goToPosition(ArmSubsystem.POSITION.DOWN);}
        else if (gamepad1.dpad_up) {robot.arm.goToPosition(ArmSubsystem.POSITION.UP);}
        else if (gamepad1.dpad_left) {robot.arm.goToPosition(ArmSubsystem.POSITION.ALMOSTUP);}
        else if (gamepad1.dpad_right) {robot.arm.goToPosition(ArmSubsystem.POSITION.CLEARTHEBARRIER);}

        robot.driveFieldCentric(forward, turn, strafe, speed);

        robot.update();

        telemetry.addData("IMU angle", String.valueOf(robot.getHeading()));
        telemetry.addData("forward",  "%.2f", forward);
        telemetry.addData("turn", "%.2f", turn);
        telemetry.addData("strafe", "%.2f", strafe);
        telemetry.addData("overall speed", "%.2f", speed);
        telemetry.addData("arm setpoint", String.valueOf(robot.arm.getSetpoint()));
    }

    /*
     * Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
    }
}
*/