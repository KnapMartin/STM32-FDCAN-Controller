# STM32-FDCAN-Controller
STM32 FDCAN Controller C++ class using HAL and CMSIS OS2.

# Usage
1. Include header
```
#include "fdcan_controller.h"
```  
1. Init a mutex, a semaphore and a FIFO0 and FIFO1 queue in global scope
```
osMutexId_t mutexCanHandle;
const osMutexAttr_t mutexCan_attributes = { .name = "mutexCan" };

osSemaphoreId_t semCanHandle;
const osSemaphoreAttr_t semCan_attributes = { .name = "semCan" };

osMessageQueueId_t queueCan0Handle;
const osMessageQueueAttr_t queueCan0_attributes = { .name = "queueCan0" };

osMessageQueueId_t queueCan1Handle;
const osMessageQueueAttr_t queueCan1_attributes = { .name = "queueCan1" };
```
2. Create "UartController" object in global scope
```
FdcanController can;
```
4. Assign UART, mutex, semaphore and queue handles to "UartController" object. Call "init" method
```
can.setHandleFdcan(&hfdcan1);
can.setHandleMutexTx(&mutexCanHandle);
can.setHandleSemTx(&semCanHandle);
can.setHandleQueueRx(&queueCan0Handle, FdcanController::Buffer::Fifo0);
can.setHandleQueueRx(&queueCan1Handle, FdcanController::Buffer::Fifo1);
if (!can.init())
{
  Error_handler();
}
```
5. Create a mutex, semaphore and both queues (of your own length...)
NOTE: Semaphore count starts at 0!
```
mutexCanHandle = osMutexNew(&mutexCan_attributes);
semCanHandle = osSemaphoreNew(1, 0, &semCan_attributes);
queueCan0Handle = osMessageQueueNew(16, sizeof(CanMsg), &queueCan0_attributes);
queueCan1Handle = osMessageQueueNew(16, sizeof(CanMsg), &queueCan1_attributes);
```
6. Define tx and both rx interrupt callbacks in main source file
```
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	  can.updateInterruptTx(hfdcan);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifoITs)
{
	  can.updateInterruptRx(hfdcan, RxFifoITs, FdcanController::Buffer::Fifo0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifoITs)
{
    can.updateInterruptRx(hfdcan, RxFifoITs, FdcanController::Buffer::Fifo1);
}
```
Now you can use "send" and "receive" methods from tasks.
