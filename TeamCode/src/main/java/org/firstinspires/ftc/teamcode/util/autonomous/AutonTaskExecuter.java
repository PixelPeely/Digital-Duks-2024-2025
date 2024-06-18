package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities.Vector;
import org.firstinspires.ftc.teamcode.util.PersistentData;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Queue;

public class AutonTaskExecuter {
    private final DukHardwareMap hMap;

    public final Queue<AutonTask> tasks = new ArrayDeque<>();
    private final List<AutonTask> synchronousTasks = new ArrayList<>();

    public AutonTaskExecuter(DukHardwareMap _hMap) {
        hMap = _hMap;
    }

    /**
     * Execute next task in the queue
     * @return True if there are no more tasks left
     */
    public boolean tick() {
        if (tasks.size() == 0) return true;
        AutonTask currentTask = tasks.peek();

        if (currentTask.runSynchronous()) {
            synchronousTasks.add(currentTask);
            tasks.remove();
            return false;
        }
        executeTask(currentTask);
        synchronousTasks.forEach(this::executeTask);
        return false;
    }

    private void executeTask(AutonTask task) {
        if (task.shouldTerminate()) {
            task.onTerminate();
            tasks.remove(task);
            synchronousTasks.remove(task);
            if (tasks.size() > 0)
                tasks.peek().initialize();
        }
        else
            task.execute();
    }

    public void renderPoints() {
        if (tasks.size() < 2) return;
        Iterator<AutonTask> futurePoints = tasks.iterator();
        futurePoints.remove();
        futurePoints.forEachRemaining(task -> {
                if (task instanceof AutonPointTask) {
                    AutonPointTask point = (AutonPointTask)task;
                    DashboardInterface.renderRobot(DukConstants.DEBUG.PATH_POINT_STROKE, point.target);
                }
        });
    }

    public void terminate() {
        hMap.driveTrain.stopMotors();
        PersistentData.available = true;
        PersistentData.pose = hMap.driveTrain.poseEstimator.getPose();
    }
}
