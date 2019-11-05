package frc.robot.Misc;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class RunCommand extends InstantCommand {
    private Runnable body;

    public RunCommand(Runnable body) {
        this.body = body;
    }

    @Override
    protected void initialize() {
        body.run();
    }
}