/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/accgyro/accgyro.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"

#include "fc/core.h"
#include "fc/tasks.h"

#include "scheduler.h"

#include "sensors/gyro_init.h"

// DEBUG_SCHEDULER, timings for:
// 0 - gyroUpdate()
// 1 - pidController()
// 2 - time spent in scheduler
// 3 - time spent executing check function

// DEBUG_SCHEDULER_DETERMINISM, requires USE_LATE_TASK_STATISTICS to be defined
// 0 - Gyro task start cycle time in 10th of a us
// 1 - ID of late task
// 2 - Amount task is late in 10th of a us
// 3 - Gyro lock skew in clock cycles

extern task_t tasks[];

static FAST_DATA_ZERO_INIT task_t *currentTask = NULL;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecRate;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecTime;

// More than this many cycles overdue will be considered late
#define DEBUG_SCHEDULER_DETERMINISM_THRESHOLD 10

int32_t schedLoopStartCycles;
static int32_t schedLoopStartMinCycles;
static int32_t schedLoopStartMaxCycles;
static uint32_t schedLoopStartDeltaDownCycles;
static uint32_t schedLoopStartDeltaUpCycles;

int32_t taskGuardCycles;
static int32_t taskGuardMinCycles;
static int32_t taskGuardMaxCycles;
static uint32_t taskGuardDeltaDownCycles;
static uint32_t taskGuardDeltaUpCycles;

static FAST_DATA_ZERO_INIT uint32_t totalWaitingTasks;
static FAST_DATA_ZERO_INIT uint32_t totalWaitingTasksSamples;

FAST_DATA_ZERO_INIT uint16_t averageSystemLoadPercent = 0;

static FAST_DATA_ZERO_INIT int taskQueuePos = 0;
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT int taskQueueSize = 0;

static bool optimizeSchedRate;

static FAST_DATA_ZERO_INIT bool gyroEnabled;

static int32_t desiredPeriodCycles;
static uint32_t lastTargetCycles;

// No need for a linked list for the queue, since items are only inserted at startup

STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT task_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue

void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

bool queueContains(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

bool queueAdd(task_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

bool queueRemove(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii+1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/*
 * Returns first item queue or NULL if queue empty
 */
FAST_CODE task_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
FAST_CODE task_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

void taskSystemLoad(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Calculate system load
    if (totalWaitingTasksSamples > 0) {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }
#if defined(SIMULATOR_BUILD)
    averageSystemLoadPercent = 0;
#endif
}

timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
    checkFuncInfo->maxExecutionTimeUs = checkFuncMaxExecutionTimeUs;
    checkFuncInfo->totalExecutionTimeUs = checkFuncTotalExecutionTimeUs;
    checkFuncInfo->averageExecutionTimeUs = checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    checkFuncInfo->averageDeltaTimeUs = checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
}

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo)
{
    taskInfo->isEnabled = queueContains(getTask(taskId));
    taskInfo->desiredPeriodUs = getTask(taskId)->desiredPeriodUs;
    taskInfo->staticPriority = getTask(taskId)->staticPriority;
    taskInfo->taskName = getTask(taskId)->taskName;
    taskInfo->subTaskName = getTask(taskId)->subTaskName;
    taskInfo->maxExecutionTimeUs = getTask(taskId)->maxExecutionTimeUs;
    taskInfo->totalExecutionTimeUs = getTask(taskId)->totalExecutionTimeUs;
    taskInfo->averageExecutionTimeUs = getTask(taskId)->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->averageDeltaTimeUs = getTask(taskId)->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->latestDeltaTimeUs = getTask(taskId)->taskLatestDeltaTimeUs;
    taskInfo->movingAverageCycleTimeUs = getTask(taskId)->movingAverageCycleTimeUs;
#if defined(USE_LATE_TASK_STATISTICS)
    taskInfo->lateCount = getTask(taskId)->lateCount;
    taskInfo->runCount = getTask(taskId)->runCount;
    taskInfo->execTime = getTask(taskId)->execTime;
#endif
}

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    if (taskId == TASK_SELF) {
        task_t *task = currentTask;
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }

    // Catch the case where the gyro loop is adjusted
    if (taskId == TASK_GYRO) {
        desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->desiredPeriodUs);
    }
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        return currentTask->taskLatestDeltaTimeUs;
    } else if (taskId < TASK_COUNT) {
        return getTask(taskId)->taskLatestDeltaTimeUs;
    } else {
        return 0;
    }
}

// Called by tasks executing what are known to be short states. Tasks should call this routine in all states
// except the one which takes the longest to execute.
void ignoreTaskStateTime()
{
    ignoreCurrentTaskExecRate = true;
    ignoreCurrentTaskExecTime = true;
}

// Called by tasks without state machines executing in what is known to be a shorter time than peak
void ignoreTaskShortExecTime()
{
    ignoreCurrentTaskExecTime = true;
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        currentTask->movingSumExecutionTimeUs = 0;
        currentTask->movingSumDeltaTimeUs = 0;
        currentTask->totalExecutionTimeUs = 0;
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        getTask(taskId)->movingSumExecutionTimeUs = 0;
        getTask(taskId)->movingSumDeltaTimeUs = 0;
        getTask(taskId)->totalExecutionTimeUs = 0;
        getTask(taskId)->maxExecutionTimeUs = 0;
    }
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->maxExecutionTimeUs = 0;
#if defined(USE_LATE_TASK_STATISTICS)
        task->lateCount = 0;
        task->runCount = 0;
#endif
    }
}

void schedulerResetCheckFunctionMaxExecutionTime(void)
{
    checkFuncMaxExecutionTimeUs = 0;
}

void schedulerInit(void)
{
    queueClear();
    queueAdd(getTask(TASK_SYSTEM));

    schedLoopStartMinCycles = clockMicrosToCycles(SCHED_START_LOOP_MIN_US);
    schedLoopStartMaxCycles = clockMicrosToCycles(SCHED_START_LOOP_MAX_US);
    schedLoopStartCycles = schedLoopStartMinCycles;
    schedLoopStartDeltaDownCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
    schedLoopStartDeltaUpCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

    taskGuardMinCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
    taskGuardMaxCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
    taskGuardCycles = taskGuardMinCycles;
    taskGuardDeltaDownCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
    taskGuardDeltaUpCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

    desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->desiredPeriodUs);

    lastTargetCycles = getCycleCounter();
}

void schedulerOptimizeRate(bool optimizeRate)
{
    optimizeSchedRate = optimizeRate;
}

inline static timeUs_t getPeriodCalculationBasis(const task_t* task)
{
    if ((task->staticPriority == TASK_PRIORITY_REALTIME)  && (optimizeSchedRate)) {
            return task->lastDesiredAt;
    }
    return task->lastExecutedAtUs;
}

FAST_CODE timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs)
{
    timeUs_t taskExecutionTimeUs = 0;

    if (selectedTask) {
        currentTask = selectedTask;
        ignoreCurrentTaskExecRate = false;
        ignoreCurrentTaskExecTime = false;
        float period = currentTimeUs - selectedTask->lastExecutedAtUs;
        selectedTask->lastExecutedAtUs = currentTimeUs;
        selectedTask->lastDesiredAt += (cmpTimeUs(currentTimeUs, selectedTask->lastDesiredAt) / selectedTask->desiredPeriodUs) * selectedTask->desiredPeriodUs;
        selectedTask->dynamicPriority = 0;

        // Execute task
        const timeUs_t currentTimeBeforeTaskCallUs = micros();
        selectedTask->taskFunc(currentTimeBeforeTaskCallUs);
        taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
        if (!ignoreCurrentTaskExecRate) {
            // Record task execution rate and max execution time
            selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastStatsAtUs);
            selectedTask->lastStatsAtUs = currentTimeUs;
            selectedTask->maxExecutionTimeUs = MAX(selectedTask->maxExecutionTimeUs, taskExecutionTimeUs);
        }
        if (!ignoreCurrentTaskExecTime) {
            // Update estimate of expected task duration
            selectedTask->movingSumExecutionTimeUs += taskExecutionTimeUs - selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
            selectedTask->movingSumDeltaTimeUs += selectedTask->taskLatestDeltaTimeUs - selectedTask->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
            }
            selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;   // time consumed by scheduler + task
            selectedTask->movingAverageCycleTimeUs += 0.05f * (period - selectedTask->movingAverageCycleTimeUs);
#if defined(USE_LATE_TASK_STATISTICS)
        selectedTask->runCount++;
#endif
    }

    return taskExecutionTimeUs;
}

#if defined(UNIT_TEST)
task_t *unittest_scheduler_selectedTask;
uint8_t unittest_scheduler_selectedTaskDynamicPriority;
uint16_t unittest_scheduler_waitingTasks;

static void readSchedulerLocals(task_t *selectedTask, uint8_t selectedTaskDynamicPriority, uint16_t waitingTasks)
{
    unittest_scheduler_selectedTask = selectedTask;
    unittest_scheduler_selectedTaskDynamicPriority = selectedTaskDynamicPriority;
    unittest_scheduler_waitingTasks = waitingTasks;
}
#endif

FAST_CODE void scheduler(void)
{
    // Cache currentTime
    const timeUs_t schedulerStartTimeUs = micros();
    timeUs_t currentTimeUs;
    uint32_t nowCycles;
    timeUs_t taskExecutionTimeUs = 0;
    task_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;
    uint16_t waitingTasks = 0;
    uint32_t nextTargetCycles = 0;
    int32_t schedLoopRemainingCycles;

    nowCycles = getCycleCounter();
    currentTimeUs = clockCyclesToMicros(nowCycles);

    bool rxTaskRan = false;
    task_t *rxTask = getTask(TASK_RX);
    if (rxTask->checkFunc(currentTimeUs, cmpTimeUs(currentTimeUs, rxTask->lastExecutedAtUs))) {
        taskExecutionTimeUs = schedulerExecuteTask(rxTask, currentTimeUs);
        rxTaskRan = true;
    }

    if (gyroEnabled && !rxTaskRan) {
        // Realtime gyro/filtering/PID tasks get complete priority
        task_t *gyroTask = getTask(TASK_GYRO);
#if defined(UNIT_TEST)
        lastTargetCycles = clockMicrosToCycles(gyroTask->lastExecutedAtUs);
#endif
        nextTargetCycles = lastTargetCycles + desiredPeriodCycles;
        schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

        if (schedLoopRemainingCycles < -desiredPeriodCycles) {
            /* A task has so grossly overrun that at entire gyro cycle has been skipped
             * This is most likely to occur when connected to the configurator via USB as the serial
             * task is non-deterministic
             * Recover as best we can, advancing scheduling by a whole number of cycles
             */
            nextTargetCycles += desiredPeriodCycles * (1 + (schedLoopRemainingCycles / -desiredPeriodCycles));
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
        }

        // Tune out the time lost between completing the last task execution and re-entering the scheduler
        if ((schedLoopRemainingCycles < schedLoopStartMinCycles) &&
            (schedLoopStartCycles < schedLoopStartMaxCycles)) {
            schedLoopStartCycles += schedLoopStartDeltaUpCycles;
        }

        // Once close to the timing boundary, poll for it's arrival
        if (schedLoopRemainingCycles < schedLoopStartCycles) {
            // Record the interval between scheduling the gyro task
            static int32_t lastRealtimeStartCycles = 0;

            if (schedLoopStartCycles > schedLoopStartMinCycles) {
                schedLoopStartCycles -= schedLoopStartDeltaDownCycles;
            }
#if !defined(UNIT_TEST)
            while (schedLoopRemainingCycles > 0) {
                nowCycles = getCycleCounter();
                schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
            }
#endif
            DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 0, clockCyclesTo10thMicros(cmpTimeCycles(nowCycles, lastRealtimeStartCycles)));
            lastRealtimeStartCycles = nowCycles;

            taskExecutionTimeUs += schedulerExecuteTask(gyroTask, currentTimeUs);

            if (gyroFilterReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_FILTER), currentTimeUs);
            }
            if (pidLoopReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_PID), currentTimeUs);
            }

            lastTargetCycles = nextTargetCycles;

#ifdef USE_GYRO_EXTI
            gyroDev_t *gyro = gyroActiveDev();

            // Bring the scheduler into lock with the gyro
            if (gyro->gyroModeSPI != GYRO_EXTI_NO_INT) {
                // Track the actual gyro rate over given number of cycle times and set the expected timebase
                static uint32_t terminalGyroRateCount = 0;
                static int32_t sampleRateStartCycles;

                if ((terminalGyroRateCount == 0)) {
                    terminalGyroRateCount = gyro->detectedEXTI + GYRO_RATE_COUNT;
                    sampleRateStartCycles = nowCycles;
                }

                if (gyro->detectedEXTI >= terminalGyroRateCount) {
                    // Calculate the number of clock cycles on average between gyro interrupts
                    uint32_t sampleCycles = nowCycles - sampleRateStartCycles;
                    desiredPeriodCycles = sampleCycles / GYRO_RATE_COUNT;
                    sampleRateStartCycles = nowCycles;
                    terminalGyroRateCount += GYRO_RATE_COUNT;
                }

                // Track the actual gyro rate over given number of cycle times and remove skew
                static uint32_t terminalGyroLockCount = 0;
                static int32_t accGyroSkew = 0;

                int32_t gyroSkew = cmpTimeCycles(nextTargetCycles, gyro->gyroSyncEXTI) % desiredPeriodCycles;
                if (gyroSkew > (desiredPeriodCycles / 2)) {
                    gyroSkew -= desiredPeriodCycles;
                }

                accGyroSkew += gyroSkew;

                if ((terminalGyroLockCount == 0)) {
                    terminalGyroLockCount = gyro->detectedEXTI + GYRO_LOCK_COUNT;
                }

                if (gyro->detectedEXTI >= terminalGyroLockCount) {
                    terminalGyroLockCount += GYRO_LOCK_COUNT;

                    // Move the desired start time of the gyroTask
                    lastTargetCycles -= (accGyroSkew/GYRO_LOCK_COUNT);
                    DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 3, clockCyclesTo10thMicros(accGyroSkew/GYRO_LOCK_COUNT));
                    accGyroSkew = 0;
                }
            }
#endif
       }
    }

    nowCycles = getCycleCounter();
    schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);


    if (!gyroEnabled || (schedLoopRemainingCycles > (int32_t)clockMicrosToCycles(CHECK_GUARD_MARGIN_US))) {
        currentTimeUs = micros();

        // Update task dynamic priorities
        for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
            if (task->staticPriority != TASK_PRIORITY_REALTIME) {
                // Task has checkFunc - event driven
                if (task->checkFunc) {
                    // Increase priority for event driven tasks
                    if (task->dynamicPriority > 0) {
                        task->taskAgeCycles = 1 + (cmpTimeUs(currentTimeUs, task->lastSignaledAtUs) / task->desiredPeriodUs);
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                        waitingTasks++;
                    } else if (task->checkFunc(currentTimeUs, cmpTimeUs(currentTimeUs, task->lastExecutedAtUs))) {
                        const uint32_t checkFuncExecutionTimeUs = cmpTimeUs(micros(), currentTimeUs);
                        DEBUG_SET(DEBUG_SCHEDULER, 3, checkFuncExecutionTimeUs);
                        checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTimeUs - checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                        checkFuncMovingSumDeltaTimeUs += task->taskLatestDeltaTimeUs - checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                        checkFuncTotalExecutionTimeUs += checkFuncExecutionTimeUs;   // time consumed by scheduler + task
                        checkFuncMaxExecutionTimeUs = MAX(checkFuncMaxExecutionTimeUs, checkFuncExecutionTimeUs);
                        task->lastSignaledAtUs = currentTimeUs;
                        task->taskAgeCycles = 1;
                        task->dynamicPriority = 1 + task->staticPriority;
                        waitingTasks++;
                    } else {
                        task->taskAgeCycles = 0;
                    }
                } else {
                    // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
                    // Task age is calculated from last execution
                    task->taskAgeCycles = (cmpTimeUs(currentTimeUs, getPeriodCalculationBasis(task)) / task->desiredPeriodUs);
                    if (task->taskAgeCycles > 0) {
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                        waitingTasks++;
                    }
                }

                if (task->dynamicPriority > selectedTaskDynamicPriority) {
                    selectedTaskDynamicPriority = task->dynamicPriority;
                    selectedTask = task;
                }
            }
        }

        totalWaitingTasksSamples++;
        totalWaitingTasks += waitingTasks;

        if (selectedTask) {
            timeDelta_t taskRequiredTimeUs = selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
#if defined(USE_LATE_TASK_STATISTICS)
            selectedTask->execTime = taskRequiredTimeUs;
#endif
            int32_t taskRequiredTimeCycles = (int32_t)clockMicrosToCycles((uint32_t)taskRequiredTimeUs);

            nowCycles = getCycleCounter();
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

            // Allow a little extra time
            taskRequiredTimeCycles += taskGuardCycles;

            if (!gyroEnabled || (taskRequiredTimeCycles < schedLoopRemainingCycles)) {
                taskExecutionTimeUs += schedulerExecuteTask(selectedTask, currentTimeUs);
                nowCycles = getCycleCounter();
                schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

                if (schedLoopRemainingCycles < taskGuardMinCycles) {
                    if (taskGuardCycles < taskGuardMaxCycles) {
                        taskGuardCycles += taskGuardDeltaUpCycles;
                    }
                } else  if (taskGuardCycles > taskGuardMinCycles) {
                    taskGuardCycles -= taskGuardDeltaDownCycles;
                }
#if defined(USE_LATE_TASK_STATISTICS)
                if (schedLoopRemainingCycles < DEBUG_SCHEDULER_DETERMINISM_THRESHOLD) {
                    DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 1, selectedTask - tasks);
                    DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 2, clockCyclesTo10thMicros(schedLoopRemainingCycles));
                    selectedTask->lateCount++ ;
                }
#endif  // USE_LATE_TASK_STATISTICS
            } else if (selectedTask->taskAgeCycles > TASK_AGE_EXPEDITE_COUNT) {
                // If a task has been unable to run, then reduce it's recorded estimated run time to ensure
                // it's ultimate scheduling
                selectedTask->movingSumExecutionTimeUs *= TASK_AGE_EXPEDITE_SCALE;
            }
        }
    }

    DEBUG_SET(DEBUG_SCHEDULER, 2, micros() - schedulerStartTimeUs - taskExecutionTimeUs); // time spent in scheduler

#if defined(UNIT_TEST)
    readSchedulerLocals(selectedTask, selectedTaskDynamicPriority, waitingTasks);
#endif
}

void schedulerEnableGyro(void)
{
    gyroEnabled = true;
}

uint16_t getAverageSystemLoadPercent(void)
{
    return averageSystemLoadPercent;
}
