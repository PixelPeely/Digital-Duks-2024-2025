package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

//All measurements are in seconds
public class TimeManager {
    private static double initTime;
    private static double currentTime = -1;
    private static double deltaTime = 1;

    private static Map<Double, Consumer<Double>> scheduledTasks = new HashMap<>();

    public static boolean hasFirstCycleRun() {
        return currentTime != -1;
    }

    public static void onCycle(double time) {
        if (hasFirstCycleRun()) deltaTime = time - currentTime;
        else initTime = time;
        currentTime = time;
        scheduledTasks.entrySet().removeIf(task -> {
            if (currentTime > task.getKey()) {
                task.getValue().accept(currentTime - task.getKey());
                return true;
            }
            return false;
        });
    }

    public static double getTime(boolean includeInit) {
        return currentTime - (includeInit ? 0 : initTime);
    }

    //Time it took to run the LAST cycle
    public static double getDeltaTime() {
        return deltaTime;
    }

    public static void scheduleFutureTask(double offset, Consumer<Double> task) {
        scheduledTasks.put(currentTime + offset, task);
    }

    public static void reset() {
        currentTime = -1;
        deltaTime = 1;
    }
}
