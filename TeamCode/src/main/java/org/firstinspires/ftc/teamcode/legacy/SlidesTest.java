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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.Robot19397;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;


/**
 * I should probably document this...
 */

@TeleOp(name="Slides Test")
public class SlidesTest extends OpMode {

    // Robot19397 robot = new Robot19397();

    @Override
    public void init() {

        // robot.init(hardwareMap);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
        if (gamepad1.left_stick_y != 0.0) {
            robot.slides.setSetpoint((int)(gamepad1.left_stick_y * 825 + 825));
            robot.slides.goToPosition(SlidesSubsystem.POSITION.CUSTOM);
        } else if (gamepad1.dpad_down) {
            robot.slides.goToPosition(SlidesSubsystem.POSITION.DOWN);
        } else if (gamepad1.dpad_right) {
            robot.slides.goToPosition(SlidesSubsystem.POSITION.CLEARTHEBARRIER);
        } else if (gamepad1.dpad_left) {
            robot.slides.goToPosition(SlidesSubsystem.POSITION.ALMOSTUP);
        } else if (gamepad1.dpad_up) {
            robot.slides.goToPosition(SlidesSubsystem.POSITION.UP);
        }

        if (gamepad1.right_stick_y != 0.0) {
            robot.arm.setSetpoint(gamepad1.right_stick_y / 2 + 0.5);
            robot.arm.goToPosition(ArmSubsystem.POSITION.CUSTOM);
        } else if (gamepad1.b) {
            robot.arm.goToPosition(ArmSubsystem.POSITION.DOWN);
        } else if (gamepad1.a) {
            robot.arm.goToPosition(ArmSubsystem.POSITION.CLEARTHEBARRIER);
        } else if (gamepad1.y) {
            robot.arm.goToPosition(ArmSubsystem.POSITION.ALMOSTUP);
        } else if (gamepad1.x) {
            robot.arm.goToPosition(ArmSubsystem.POSITION.UP);
        }

         */

        // telemetry.addData("SlidesPosition", robot.slides.getPosition());

        // robot.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}