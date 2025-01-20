/*
 * fdcan_controller.cpp
 *
 *  Created on: Jan 17, 2025
 *      Author: knap-linux
 */

#include "fdcan_controller.h"

FdcanController::FdcanController()
	: m_hfdcan{nullptr}
	, m_queueCanHandleFifo0{nullptr}
	, m_queueCanHandleFifo1{nullptr}
	, m_mutexCanHandle{nullptr}
	, m_semCanHandle{nullptr}
{

}

FdcanController::~FdcanController()
{

}

void FdcanController::setHandleFdcan(FDCAN_HandleTypeDef *hfdcan)
{
	m_hfdcan = hfdcan;
}

FdcanController::State FdcanController::setHandleQueue(osMessageQueueId_t *queueCanHandle, const Buffer bufferType)
{
	if (bufferType == Buffer::Fifo0)
	{
		m_queueCanHandleFifo0 = queueCanHandle;
	}
	else if (bufferType == Buffer::Fifo1)
	{
<<<<<<< HEAD
		m_queueCanHandleFifo1 = queueCanHandle;
=======
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
		osMutexRelease(*m_mutexTx);
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
>>>>>>> origin/main
	}
	else
	{
		return State::ErrorHandleQueue;
	}

	return State::Ok;
}

void FdcanController::setHandleMutex(osMutexId_t *mutexCanHandle)
{
	m_mutexCanHandle = mutexCanHandle;
}

void FdcanController::setHandleSem(osSemaphoreId_t *semCanHandle)
{
	m_semCanHandle = semCanHandle;
}

FdcanController::State FdcanController::init()
{
	if (m_hfdcan == nullptr) return State::ErrorInit;
	if ((m_queueCanHandleFifo0 == nullptr) &&
		(m_queueCanHandleFifo1 == nullptr)) return State::ErrorInit;
	if (m_mutexCanHandle == nullptr) return State::ErrorInit;
	if (m_semCanHandle == nullptr) return State::ErrorInit;

	if (HAL_FDCAN_Start(m_hfdcan) != HAL_OK)
	{
		return State::ErrorInit;
	}

	if (HAL_FDCAN_ActivateNotification(m_hfdcan,
		FDCAN_IT_TX_COMPLETE | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
		FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
	{
		return State::ErrorInit;
	}

	return State::Ok;
}

FdcanController::State FdcanController::send(const FdcanMsg msg)
{
	if (osMutexAcquire(*m_mutexCanHandle, osWaitForever) != osOK)
	{
		return State::ErrorSend;
	}
	if (HAL_FDCAN_AddMessageToTxFifoQ(m_hfdcan, &msg.txHeader, msg.data) != HAL_OK)
	{
		osMutexRelease(*m_mutexCanHandle);
		return State::ErrorSend;
	}
	if (osSemaphoreAcquire(*m_semCanHandle, osWaitForever) != osOK)
	{
		osMutexRelease(*m_mutexCanHandle);
		return State::ErrorSend;
	}
	if (osMutexRelease(*m_mutexCanHandle) != osOK)
	{
		return State::ErrorSend;
	}

	return State::Ok;
}

FdcanController::State FdcanController::receive(FdcanMsg *msg, const Buffer bufferType)
{
	osMessageQueueId_t *queueHandle = nullptr;

	if (bufferType == Buffer::Fifo0)
	{
		queueHandle = m_queueCanHandleFifo0;
	}
	else if (bufferType == Buffer::Fifo1)
	{
		queueHandle = m_queueCanHandleFifo1;
	}
	else
	{
		return State::ErrorReceive;
	}

	if (osMessageQueueGet(*queueHandle, msg, nullptr, osWaitForever) != osOK)
	{
		return State::ErrorReceive;
	}

	return State::Ok;
}

FdcanController::State FdcanController::updateInterruptTx(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan->Instance == m_hfdcan->Instance)
	{
		if (osSemaphoreRelease(*m_semCanHandle) != osOK)
		{
			return State::ErrorIsrTx;
		}
	}

	return State::Ok;
}

FdcanController::State FdcanController::updateInterruptRx(FDCAN_HandleTypeDef *hfdcan, uint32_t isrType)
{
	if (hfdcan->Instance == m_hfdcan->Instance)
	{
		if ((isrType & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) ||
			(isrType & FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
		{
			FdcanMsg msg;
			uint32_t fifo = (isrType & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;
			osMessageQueueId_t *queueHandle = (fifo == FDCAN_RX_FIFO0) ? m_queueCanHandleFifo0 : m_queueCanHandleFifo1;
			if (HAL_FDCAN_GetRxMessage(m_hfdcan, fifo, &msg.rxHeader, msg.data) != HAL_OK)
			{
				return State::ErrorIsrRx;
			}
			if (osMessageQueuePut(*queueHandle, &msg, 0, 0) != osOK)
			{
				return State::ErrorIsrRx;
			}
		}
	}

	return State::Ok;
}

FdcanController::State FdcanController::setFilter(FDCAN_FilterTypeDef *filter)
{
	if (HAL_FDCAN_ConfigFilter(m_hfdcan, filter) != HAL_OK)
	{
		return State::ErrorFilter;
	}

	return State::Ok;
}
