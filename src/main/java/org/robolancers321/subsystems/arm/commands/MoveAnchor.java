package org.robolancers321.subsystems.arm.commands;

import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveAnchor extends CommandBase{
    private Arm arm;
    private double setpoint;

    public MoveAnchor(Arm arm, Constants.RawArmSetpoints setpoint){
        this.arm = arm;
        this.setpoint = setpoint.anchor;
    }

    @Override
    public void initialize(){
        arm.setAnchorGoal(setpoint);
    }

    @Override
    public boolean isFinished(){
        return arm.getAnchorAtSetpoint();
    }

}
