package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends InstantCommand {

    private Drivetrain drivetrain;

    public AutoBalance(Drivetrain d) {
        super();
        drivetrain = d;
    }


    @Override
    public void execute() {
        if (Math.abs(drivetrain.getPidgeonPitch())>Math.abs(/*pidgeon roll */0)) {
            new AutoBalanceFrontBack(drivetrain);
        } else {
            new AutoBalanceLeftRight(drivetrain);
        }
    }
}
