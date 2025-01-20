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

void FdcanController::setHandleQueue(osMessageQueueId_t *queueCanHandle, const Buffer bufferType)
{
	if (bufferType == Buffer::Fifo0)
	{
		m_queueCanHandleFifo0 = queueCanHandle;
	}
	else if (bufferType == Buffer::Fifo1)
	{
		m_queueCanHandleFifo1 = queueCanHandle;
	}
	else
	{
		; // TODO: implement error
	}
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
	if (m_hfdcan == nullptr) return State::Error;
	if ((m_queueCanHandleFifo0 == nullptr) &&
		(m_queueCanHandleFifo1 == nullptr)) return State::Error;
	if (m_mutexCanHandle == nullptr) return State::Error;
	if (m_semCanHandle == nullptr) return State::Error;

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
	if (bufferType == Buffer::Fifo0)
	{
		if (osMessageQueueGet(*m_queueCanHandleFifo0, msg, nullptr, osWaitForever) != osOK)
		{
			return State::ErrorReceive;
		}
	}
	else if (bufferType == Buffer::Fifo1)
	{
		if (osMessageQueueGet(*m_queueCanHandleFifo1, msg, nullptr, osWaitForever) != osOK)
		{
			return State::ErrorReceive;
		}
	}
	else
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
		if (isrType & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
		{
			FdcanMsg msg;
			if (HAL_FDCAN_GetRxMessage(m_hfdcan, FDCAN_RX_FIFO0, &msg.rxHeader, msg.data) != HAL_OK)
			{
				return State::ErrorIsrRx;
			}
			if (osMessageQueuePut(*m_queueCanHandleFifo0, &msg, 0, 0) != osOK)
			{
				return State::ErrorIsrRx;
			}
		}
		else if (isrType & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
		{
			FdcanMsg msg;
			if (HAL_FDCAN_GetRxMessage(m_hfdcan, FDCAN_RX_FIFO1, &msg.rxHeader, msg.data) != HAL_OK)
			{
				return State::ErrorIsrRx;
			}
			if (osMessageQueuePut(*m_queueCanHandleFifo1, &msg, 0, 0) != osOK)
			{
				return State::ErrorIsrRx;
			}
		}
	}

	return State::Ok;
}

FdcanController::State FdcanController::setFilter(FDCAN_FilterTypeDef filter)
{
	if (HAL_FDCAN_ConfigFilter(m_hfdcan, &filter) != HAL_OK)
	{
		return State::ErrorFilter;
	}

	return State::Ok;
}
