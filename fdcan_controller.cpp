/*
 * fdcan_controller.cpp
 *
 *  Created on: Jan 8, 2025
 *      Author: knap-linux
 */

#include "fdcan_controller.h"

FdcanController::FdcanController()
	: m_hfdcan{ nullptr }
	, m_semTx{ nullptr }
	, m_queueRxFifo0{ nullptr }
	, m_queueRxFifo1{ nullptr }
	, m_mutexTx{ nullptr }
{

}

FdcanController::~FdcanController()
{

}

void FdcanController::setHandleFdcan(FDCAN_HandleTypeDef *hfdcan)
{
	m_hfdcan = hfdcan;
}

void FdcanController::setHandleSemTx(osSemaphoreId_t *semTx)
{
	m_semTx = semTx;
}

bool FdcanController::setHandleQueueRx(osMessageQueueId_t *queueRx,
		Buffer bufferType)
{
	if (bufferType == Buffer::Fifo0)
	{
		m_queueRxFifo0 = queueRx;
		return true;
	}
	else if (bufferType == Buffer::Fifo1)
	{
		m_queueRxFifo1 = queueRx;
		return true;
	}

	return false;
}

void FdcanController::setHandleMutexTx(osMutexId_t *mutexTx)
{
	m_mutexTx = mutexTx;
}

bool FdcanController::init()
{
	if (m_hfdcan == nullptr)
		return false;
	if (m_semTx == nullptr)
		return false;
	if ((m_queueRxFifo0 == nullptr) && (m_queueRxFifo1 == nullptr))
		return false;
	if (m_mutexTx == nullptr)
		return false;

	if (HAL_FDCAN_ActivateNotification(m_hfdcan,
			FDCAN_IT_TX_COMPLETE | FDCAN_IT_RX_FIFO0_NEW_MESSAGE
					| FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
			FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
	{
		return false;
	}
	if (HAL_FDCAN_Start(m_hfdcan) != HAL_OK)
	{
		return false;
	}

	return true;
}

bool FdcanController::send(const CanMsg &msg)
{
	if (osMutexAcquire(*m_mutexTx, osWaitForever) != osOK)
	{
		return false;
	}
	if (HAL_FDCAN_AddMessageToTxFifoQ(m_hfdcan, &msg.txHeader, msg.data)
			!= HAL_OK)
	{
		return false;
	}
	if (osSemaphoreAcquire(*m_semTx, osWaitForever) != osOK)
	{
		return false;
	}
	if (osMutexRelease(*m_mutexTx) != osOK)
	{
		return false;
	}

	return true;
}

CanRxResult FdcanController::receive(Buffer bufferType)
{
	CanRxResult result;
	result.status = CanRxStatus::Error;
	if ((bufferType != Buffer::Fifo0) && (bufferType != Buffer::Fifo1))
	{
		result.status = CanRxStatus::InvalidBuffer;
		return result;
	}

	osMessageQueueId_t *queue =
			(bufferType == Buffer::Fifo0) ? m_queueRxFifo0 : m_queueRxFifo1;
	if ((queue == nullptr) || (*queue == nullptr))
	{
		result.status = CanRxStatus::Error;
		return result;
	}

	osStatus_t status = osMessageQueueGet(*queue, &result.message, nullptr,
	osWaitForever);
	switch (status)
	{
	case osOK:
		result.status = CanRxStatus::Ok;
		break;
	case osErrorTimeout:
		result.status = CanRxStatus::Timeout;
		break;
	case osErrorResource:
		result.status = CanRxStatus::QueueEmpty;
		break;
	default:
		result.status = CanRxStatus::Error;
		break;
	}

	return result;
}

bool FdcanController::updateInterruptTx(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan->Instance == m_hfdcan->Instance)
	{
		if (osSemaphoreRelease(*m_semTx) != osOK)
		{
			return false;
		}
	}

	return true;
}

bool FdcanController::updateInterruptRx(FDCAN_HandleTypeDef *hfdcan,
		uint32_t isrType, Buffer bufferType)

{
	if (hfdcan->Instance != m_hfdcan->Instance)
	{
		return false;
	}

	if ((bufferType != Buffer::Fifo0) && (bufferType != Buffer::Fifo1))
	{
		return false;
	}

	uint32_t fifoInterrupt;
	uint32_t fifoIndex;
	osMessageQueueId_t *queue;

	if (bufferType == Buffer::Fifo0)
	{
		fifoInterrupt = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
		fifoIndex = FDCAN_RX_FIFO0;
		queue = m_queueRxFifo0;
	}
	else
	{
		fifoInterrupt = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
		fifoIndex = FDCAN_RX_FIFO1;
		queue = m_queueRxFifo1;
	}

	if (!(isrType & fifoInterrupt))
	{
		return false;
	}

	CanMsg msg;
	if (HAL_FDCAN_GetRxMessage(m_hfdcan, fifoIndex, &msg.rxHeader, msg.data)
			!= HAL_OK)
	{
		return false;
	}

	if (osMessageQueuePut(*queue, &msg, 0, 0) != osOK)
	{
		return false;
	}

	return true;
}

bool FdcanController::setFilter(FDCAN_FilterTypeDef *filter)
{
	if (HAL_FDCAN_ConfigFilter(m_hfdcan, filter) != HAL_OK)
	{
		return false;
	}

	return true;
}
