package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.Robot19397;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
/*
@Autonomous(name = "Stupidly Simple")
public class StupidlySimpleAuto extends OpMode {

    enum STATE {
        DRIVINGBACKWARD,
        DUMPING,
        STRAFING,
        END
    }

    STATE state = STATE.DRIVINGBACKWARD;
    private boolean isRed = true;
    private boolean aAlreadyDown = false;
    private final double drivingBackwardEndTime = 1000;
    private final double dumpingEndTime = 2000;
    private final double strafingEndTime = 3500;
    Robot19397 robot = new Robot19397(new ElapsedTime());
    ElapsedTime timer;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.arm.goToPosition(ArmSubsystem.POSITION.ALMOSTUP);
        telemetry.addData("Are we on the red alliance? (toggle with a)", isRed);
    }

    @Override
    public void init_loop() {
        if (gamepad1.a != aAlreadyDown) {
            aAlreadyDown = gamepad1.a;
            if (aAlreadyDown) {
                isRed = !isRed;
            }
        }
        telemetry.addData("Front Joystick Reversed? (toggle with a)", isRed);
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        switch (state) {
            case DRIVINGBACKWARD:
                robot.driveSimple(-0.3, 0, 0, 1);
                if (timer.milliseconds() >= drivingBackwardEndTime) {
                    state = STATE.DUMPING;
                }
                break;
            case DUMPING:
                robot.driveSimple(0, 0, 0, 0);
                robot.arm.goToPosition(ArmSubsystem.POSITION.UP);
                if (timer.milliseconds() >= dumpingEndTime) {
                    state = STATE.STRAFING;
                }
                break;
            case STRAFING:
                robot.driveSimple(0, 0, isRed ? -0.5 : 0.5, 1);
                robot.arm.goToPosition(ArmSubsystem.POSITION.CLEARTHEBARRIER);
                if (timer.milliseconds() >= strafingEndTime) {
                    state = STATE.END;
                }
                break;
            case END:
                robot.driveSimple(0, 0, 0, 0);
                break;
        }
    }
}
 */