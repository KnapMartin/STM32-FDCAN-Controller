/*
 * fdcan_controller.h
 *
 *  Created on: Jan 17, 2025
 *      Author: knap-linux
 */

#ifndef FDCAN_CONTROLLER_H_
#define FDCAN_CONTROLLER_H_

#include "main.h"
#include "cmsis_os.h"

constexpr uint8_t FDCAN_MSG_LEN{8};

struct FdcanMsg
{
	FDCAN_RxHeaderTypeDef rxHeader;
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t data[FDCAN_MSG_LEN];
};

class FdcanController
{
public:
	FdcanController();
	virtual ~FdcanController();

	enum class State
	{
		Ok = 0,
		Error = 1,
		ErrorInit,
		ErrorSend,
		ErrorReceive,
		ErrorIsrTx,
		ErrorIsrRx,
		ErrorFilter
	};

	enum class Buffer
	{
		None = 0,
		Fifo0,
		Fifo1,
	};

	void setHandleFdcan(FDCAN_HandleTypeDef *hfdcan);
	void setHandleQueue(osMessageQueueId_t *queueCanHandle, const Buffer bufferType);
	void setHandleMutex(osMutexId_t *mutexCanHandle);
	void setHandleSem(osSemaphoreId_t *semCanHandle);

	State init();
	State send(const FdcanMsg msg);
	State receive(FdcanMsg *msg, const Buffer bufferType);

	State updateInterruptTx(FDCAN_HandleTypeDef *hfdcan);
	State updateInterruptRx(FDCAN_HandleTypeDef *hfdcan, uint32_t isrType);

	FdcanController::State setFilter(FDCAN_FilterTypeDef filter);

private:
	FDCAN_HandleTypeDef *m_hfdcan;
	osMessageQueueId_t *m_queueCanHandleFifo0;
	osMessageQueueId_t *m_queueCanHandleFifo1;
	osMutexId_t *m_mutexCanHandle;
	osSemaphoreId_t *m_semCanHandle;

};

#endif /* FDCAN_CONTROLLER_H_ */
