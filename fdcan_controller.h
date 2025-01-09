/*
 * fdcan_controller.h
 *
 *  Created on: Jan 8, 2025
 *      Author: knap-linux
 */

#ifndef FDCAN_CONTROLLER_H_
#define FDCAN_CONTROLLER_H_

#include "main.h"
#include "cmsis_os.h"

constexpr uint8_t canMsgLen
{ 8 };

struct CanMsg
{
	uint8_t data[canMsgLen];
	uint8_t length
	{ canMsgLen };
	FDCAN_TxHeaderTypeDef txHeader
	{ };
	FDCAN_RxHeaderTypeDef rxHeader
	{ };
};

enum class CanRxStatus
{
	Ok = 0, InvalidBuffer, QueueEmpty, Timeout, Error
};

struct CanRxResult
{
	CanRxStatus status;
	CanMsg message;
};

class FdcanController
{
public:
	FdcanController();
	virtual ~FdcanController();

	enum class Buffer
	{
		None, Fifo0, Fifo1
	};

	void setHandleFdcan(FDCAN_HandleTypeDef *hfdcan);
	void setHandleSemTx(osSemaphoreId_t *semTx);
	bool setHandleQueueRx(osMessageQueueId_t *queueRx, Buffer bufferType);
	void setHandleMutexTx(osMutexId_t *mutexTx);

	bool init();
	bool send(const CanMsg &msg);
	CanRxResult receive(Buffer bufferType);

	bool updateInterruptRx(FDCAN_HandleTypeDef *hfdcan, uint32_t isrType,
			Buffer bufferType);
	bool updateInterruptTx(FDCAN_HandleTypeDef *hfdcan);

	bool setFilter(FDCAN_FilterTypeDef *filter);

private:
	FDCAN_HandleTypeDef *m_hfdcan;
	osSemaphoreId_t *m_semTx;
	osMessageQueueId_t *m_queueRxFifo0;
	osMessageQueueId_t *m_queueRxFifo1;
	osMutexId_t *m_mutexTx;

};

#endif /* FDCAN_CONTROLLER_H_ */
