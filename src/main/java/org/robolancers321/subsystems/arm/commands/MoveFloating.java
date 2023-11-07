package org.robolancers321.subsystems.arm.commands;

import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveFloating extends CommandBase{
    private Arm arm;
    private double setpoint;

    public MoveFloating(Arm arm, Constants.RawArmSetpoints setpoint){
        this.arm = arm;
        this.setpoint = setpoint.floating;
    }

    @Override
    public void initialize(){
        arm.setFloatingGoal(setpoint);
    }

    @Override
    public boolean isFinished(){
        return arm.getFloatingAtSetpoint();
    }

}
