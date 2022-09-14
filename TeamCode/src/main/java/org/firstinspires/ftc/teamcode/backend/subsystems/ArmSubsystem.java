package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.utilities.DoubledServo;

@Config
public class ArmSubsystem extends Subsystem implements PositionControlled {

    protected DcMotor motor;

    protected double targetPosition = 1.0;
    private final int TOTALTICKS = 269;

    public ArmSubsystem (ElapsedTime aTimer, HardwareMap ahwMap) {
        super(aTimer, ahwMap);
        motor = ahwMap.get(DcMotor.class, "ArmMotor" );
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setTargetPosition(0.0);
    }

    public void setTargetPosition(double target) {
        if (targetPosition == target) {return;}
        targetPosition = target;
        motor.setTargetPosition((int)(targetPosition*this.TOTALTICKS));
    }

    public double getPosition() {return targetPosition;}

}
