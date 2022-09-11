package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.utilities.DoubledServo;

@Config
public class ArmSubsystem extends Subsystem implements PositionControlled {

    // Legacy servo setpoints
    // private final int  upPosition              = 0.475;
    // private final int  almostUpPosition        = 0.325;
    // private final int  clearTheBarrierPosition = 0.075;

    protected Servo leftServo;
    protected Servo rightServo;

    protected double targetPosition = 1.0;

    public ArmSubsystem (ElapsedTime aTimer, HardwareMap ahwMap) {
        super(aTimer, ahwMap);
        leftServo  = ahwMap.get(Servo.class, "LeftArmServo" );
        rightServo = ahwMap.get(Servo.class, "RightArmServo");
        rightServo.setDirection(Servo.Direction.REVERSE);
        setTargetPosition(0.0);
    }

    public void setTargetPosition(double target) {
        if (targetPosition == target) {return;}
        targetPosition = target;
        leftServo.setPosition(targetPosition+0.02);
        rightServo.setPosition(targetPosition+0.03);
    }

    public double getPosition() {return targetPosition;}

}
