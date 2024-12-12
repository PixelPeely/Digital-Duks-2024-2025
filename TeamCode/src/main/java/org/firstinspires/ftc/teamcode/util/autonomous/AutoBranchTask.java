package org.firstinspires.ftc.teamcode.util.autonomous;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class AutoBranchTask implements AutoTask {
    Supplier<List<AutoTask>> branchSupplier;

    public AutoBranchTask(Supplier<List<AutoTask>> _branchSupplier) {
        branchSupplier = _branchSupplier;
    }

    public List<AutoTask> resolve() {
        List<AutoTask> branch = branchSupplier.get();
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
