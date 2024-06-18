package org.firstinspires.ftc.teamcode.util.autonomous;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class AutonBranchTask implements AutonTask {
    Supplier<List<AutonTask>> branchSupplier;

    public AutonBranchTask(Supplier<List<AutonTask>> _branchSupplier) {
        branchSupplier = _branchSupplier;
    }

    public List<AutonTask> resolve() {
        List<AutonTask> branch = branchSupplier.get();
        Collections.reverse(branch);
        return branch;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean shouldTerminate() {
        return true;
    }

    @Override
    public void onTerminate() {}

    @Override
    public boolean runSynchronous() {
        return false;
    }
}
