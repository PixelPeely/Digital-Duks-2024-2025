package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutonBranchTask;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

public abstract class DukAutonomous extends DukOpMode{
    private final Deque<AutonTask> tasks = new ArrayDeque<>();
    private final List<AutonTask> synchronousTasks = new ArrayList<>();

    @Override
    public void setup() {
        buildAutonomous();
        _hardwareMap.driveTrain.pursueHeading = true;
        _hardwareMap.driveTrain.pursuePosition = true;
    }

    @Override
    public void start() {}

    @Override
    public void preTick() {
        if (tasks.size() == 0) stop();
        AutonTask currentTask = tasks.peekFirst();

        if (currentTask.runSynchronous()) {
            synchronousTasks.add(currentTask);
            tasks.removeFirst();
        }
        executeTask(currentTask);
        synchronousTasks.forEach(this::executeTask);
    }

    @Override
    public void postTick() {}

    @Override
    public void stop() {
        PersistentData.available = true;
        PersistentData.pose = _hardwareMap.driveTrain.poseEstimator.getPose();
        super.stop();
    }

    private void executeTask(AutonTask task) {
        if (task.shouldTerminate()) {
            task.onTerminate();
            tasks.remove(task);//TODO maybe just change to removeFirst()?

            if (task instanceof AutonBranchTask)//TODO TEST!!
                for (AutonTask resolvedTask : ((AutonBranchTask) task).resolve())
                    tasks.offerFirst(resolvedTask);

            synchronousTasks.remove(task);
            if (tasks.size() > 0)
                tasks.peekFirst().initialize();
        }
        else
            task.execute();
    }

    public void register(AutonTask task) {
        tasks.offerLast(task);
    }

    public List<AutonTask> branch (AutonTask... tasks) {
        return Arrays.asList(tasks);
    }

    public abstract void buildAutonomous();
}
