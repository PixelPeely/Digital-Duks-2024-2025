package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoBranchTask;
import org.firstinspires.ftc.teamcode.util.autonomous.AutoWaitTask;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

public abstract class DukAutonomous extends DukOpMode{
    private final Deque<AutoTask> tasks = new ArrayDeque<>();
    private final List<AutoTask> synchronousTasks = new ArrayList<>();

    @Override
    public void init() {
        super.init();
        tasks.add(new AutoWaitTask(0));
        buildAutonomous();
    }

    @Override
    public void start() {}

    @Override
    public void preTick() {
        if (tasks.size() == 0) {
            Logger.addEntry(new Logger.LogEntry("Task list empty; requesting OpMode stop", Logger.LOG_TYPE.INFO));
            requestOpModeStop();
            return;
        }
        AutoTask currentTask = tasks.peekFirst();

        if (currentTask.runSynchronous()) {
            synchronousTasks.add(currentTask);
            tasks.removeFirst();
            return;
        }
        executeTask(currentTask);
        synchronousTasks.forEach(this::executeTask);
    }

    @Override
    public void postTick() {}

    @Override
    public void stop() {
        super.stop();
        PersistentData.available = true;
        PersistentData.pose = _hardwareMap.driveTrain.poseEstimator.getPose();
    }

    private void executeTask(AutoTask task) {
        if (task.shouldTerminate()) {
            Logger.addEntry(new Logger.LogEntry("Terminated " + task.getClass().getSimpleName(), Logger.LOG_TYPE.INFO));
            task.onTerminate();
            tasks.remove(task);

            if (task instanceof AutoBranchTask)
                for (AutoTask resolvedTask : ((AutoBranchTask) task).resolve()) {
                    tasks.offerFirst(resolvedTask);
                    Logger.addEntry(new Logger.LogEntry("\tEnqueuing " + resolvedTask.getClass().getSimpleName(), Logger.LOG_TYPE.INFO));
                }

            synchronousTasks.remove(task);
            if (tasks.size() > 0)
                tasks.peekFirst().initialize();
        }
        else
            task.execute();
    }


    /**
     * Enqueue a task for the sequential task executor to run
     * @param task The task to enqueue
     */
    public void register(AutoTask task) {
        tasks.offerLast(task);
    }

    /**
     * Short-hand for creating a list to serve as a sequential task branch
     * @param tasks Tasks that constitute the branch, in execution order
     * @return A list representing the tasks
     */
    public List<AutoTask> branch (AutoTask... tasks) {
        return Arrays.asList(tasks);
    }

    public abstract void buildAutonomous();
}
