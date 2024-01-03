package frc.robot.libs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.libs.BaseMechanism.BaseMechanism;
import frc.robot.libs.BaseMechanism.SparkMAXMechanism;

public class BaseMechanismPowerBrake extends Command {

    BaseMechanism mech;
    double holdPos;
    public BaseMechanismPowerBrake(BaseMechanism mech) {
        this.mech = mech;
        addRequirements((Subsystem) this.mech);
    }

    @Override
    public void initialize() {
        mech.setHoldPosition(mech.getPosition());
    }



    @Override
    public void execute() {
        mech.setPosition(mech.getHoldPosition());
    }

}
