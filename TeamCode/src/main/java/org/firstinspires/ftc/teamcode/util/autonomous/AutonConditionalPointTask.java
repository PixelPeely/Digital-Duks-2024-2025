package org.firstinspires.ftc.teamcode.util.autonomous;

import java.util.function.Supplier;

public class AutonConditionalPointTask implements AutonTask {
    private final Supplier<AutonPointTask> pointTaskSupplier;

    public AutonConditionalPointTask(Supplier<AutonPointTask> _pointTaskSupplier) {
        pointTaskSupplier = _pointTaskSupplier;
    }

    @Override
    public void initialize() {
        pointTaskSupplier.get().initialize();
    }

    @Override
    public void execute() {
        pointTaskSupplier.get().execute();
    }

    @Override
    public boolean shouldTerminate() {
        return pointTaskSupplier.get().shouldTerminate();
    }

    @Override
    public void onTerminate() {
        pointTaskSupplier.get().onTerminate();
    }

    @Override
    public boolean runAsynchronous() {
        return pointTaskSupplier.get().runAsynchronous();
    }
}
