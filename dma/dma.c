#include "dma.h"


static inline int dma_rx_init(dma_t*);
static inline int dma_tx_init(dma_t*);

static void dma_rx_intr_handler(void*);
static void dma_tx_intr_handler(void*);


int dma_init(dma_t* dma) {
	dma->config = XAxiDma_LookupConfig(dma->params.device_id);
	if (!dma->config)
		return XST_NO_DATA;

	int status = XAxiDma_CfgInitialize(dma->instance, dma->config);
	if (status != XST_SUCCESS)
		return status;

	if (!XAxiDma_HasSg(dma->instance))
		return XST_NO_DATA;

	status = dma_rx_init(dma);
	if (status != XST_SUCCESS)
		return status;

	status = dma_tx_init(dma);
	if (status != XST_SUCCESS)
		return status;

    return XST_SUCCESS;
}


int dma_rx_init(dma_t* dma) {
	XAxiDma_BdRing* rx_ring = XAxiDma_GetRxRing(dma->instance);
    XAxiDma_BdRingIntDisable(rx_ring, XAXIDMA_IRQ_ALL_MASK);

	int status = XAxiDma_BdRingCreate(rx_ring,
								     (u32)dma->params.rx_descriptors_addr,
								     (u32)dma->params.rx_descriptors_addr,
								     DMA_BD_ALIGN,
								     dma->params.descriptors_count);
	if (status != XST_SUCCESS)
		return status;

    XAxiDma_Bd bd_template;
    XAxiDma_BdClear(&bd_template);

    status = XAxiDma_BdRingClone(rx_ring, &bd_template);
    if (status != XST_SUCCESS)
    	return status;

    int free_bd_cnt = XAxiDma_BdRingGetFreeCnt(rx_ring);
    XAxiDma_Bd* pointer_bd;
    status = XAxiDma_BdRingAlloc(rx_ring, free_bd_cnt, &pointer_bd);
    if (status != XST_SUCCESS)
    	return status;

    XAxiDma_Bd* current_bd = pointer_bd;
    UINTPTR rx_buff_ptr = (UINTPTR)dma->params.rx_buff_addr;
    for (int i = 0; i < free_bd_cnt; i++) {
    	UINTPTR offset = rx_buff_ptr + (i * dma->params.descriptor_buff_size);
    	status = XAxiDma_BdSetBufAddr(current_bd, offset);
    	if (status != XST_SUCCESS)
    		return status;

    	status = XAxiDma_BdSetLength(current_bd,
    								 dma->params.descriptor_buff_size,
									 rx_ring->MaxTransferLen);
    	if (status != XST_SUCCESS)
    		return status;

    	current_bd = (XAxiDma_Bd*)XAxiDma_BdRingNext(rx_ring, current_bd);
    }

    status = XAxiDma_BdRingToHw(rx_ring, free_bd_cnt, pointer_bd);
    if (status != XST_SUCCESS)
    	return status;

    XAxiDma_BdRingIntEnable(rx_ring, XAXIDMA_IRQ_ALL_MASK);
    return XAxiDma_BdRingStart(rx_ring);
}


int dma_tx_init(dma_t* dma) {
	XAxiDma_BdRing* tx_ring = XAxiDma_GetTxRing(dma->instance);
    XAxiDma_BdRingIntDisable(tx_ring, XAXIDMA_IRQ_ALL_MASK);

	int status = XAxiDma_BdRingCreate(tx_ring,
								     (u32)dma->params.tx_descriptors_addr,
								     (u32)dma->params.tx_descriptors_addr,
								     DMA_BD_ALIGN,
								     dma->params.descriptors_count);
    if (status != XST_SUCCESS)
    	return status;

    XAxiDma_Bd bd_template;
	XAxiDma_BdClear(&bd_template);
    status = XAxiDma_BdRingClone(tx_ring, &bd_template);
    if (status != XST_SUCCESS) {
    	return status;
    }

    XAxiDma_BdRingIntEnable(tx_ring, XAXIDMA_IRQ_ALL_MASK);
	return XAxiDma_BdRingStart(tx_ring);
}


/**
 * @brief Set interrupts, set interrupt handler, priority,e tc
 * @param[inout] dma 				  - dma handler
 * @param[out]   interrupt_controller - interrupt handler
 * @param[in]    context 			  - contains dma interrupt context
 *
 * @return XST_SUCCESS - interrupt set successful
 * @return other	   - can't set interrupt
 */
int dma_setup_interrupts(dma_t* dma, XScuGic* interrupt_controller,
						 dma_interrupt_handler_t* context) {
	XScuGic_SetPriorityTriggerType(interrupt_controller,
						           dma->params.interrupt_rx_vec_id,
						           dma->params.rx_interrupt_priority, 0x3);
    XScuGic_SetPriorityTriggerType(interrupt_controller,
       							   dma->params.interrupt_tx_vec_id,
								   dma->params.tx_interrupt_priority, 0x3);
	int status = XScuGic_Connect(interrupt_controller,
								 dma->params.interrupt_rx_vec_id,
								 dma_rx_intr_handler, context);
	if (status != XST_SUCCESS)
		return status;

	status = XScuGic_Connect(interrupt_controller,
							 dma->params.interrupt_tx_vec_id,
						 	 dma_tx_intr_handler, context);
	if (status != XST_SUCCESS)
		return status;

	XScuGic_Enable(interrupt_controller, dma->params.interrupt_rx_vec_id);
	XScuGic_Enable(interrupt_controller, dma->params.interrupt_tx_vec_id);

	return XST_SUCCESS;
}


/**
 * @brief Transmit data from AXI DMA
 * @param[inout] dma 	- AXI DMA controller
 * @param[in]    buffer - data for sending
 * @param[in] 	 size   - the size of buffer
 *
 * @return  XST_SUCCESS - transfer was successful
 * @return -XST_FAILURE - if size of buffer is more than in descriptor
 * @return other 		- other DMA errors
 */
int dma_transmit(dma_t* dma, const uint8_t* buffer, uint32_t size) {
	if (size > dma->params.descriptor_buff_size)
		return -XST_FAILURE;

	XAxiDma_BdRing* tx_ring = XAxiDma_GetTxRing(dma->instance);
	XAxiDma_Bd* pointer_bd  = NULL;

	int status = XAxiDma_BdRingAlloc(tx_ring, DMA_TX_BD_CNT, &pointer_bd);
	if (status != XST_SUCCESS)
		return status;

	UINTPTR buff_addr = (UINTPTR)dma->params.tx_buff_addr;
	memmove(dma->params.tx_buff_addr, buffer, size);
	Xil_DCacheFlushRange((INTPTR)buff_addr, size);

	status = XAxiDma_BdSetBufAddr(pointer_bd, buff_addr);
	if (status != XST_SUCCESS)
		return status;

	status = XAxiDma_BdSetLength(pointer_bd, size, tx_ring->MaxTransferLen);
	if (status != XST_SUCCESS)
		return status;

	XAxiDma_BdSetCtrl(pointer_bd, XAXIDMA_BD_CTRL_TXSOF_MASK |
								  XAXIDMA_BD_CTRL_TXEOF_MASK);
	status = XAxiDma_BdRingToHw(tx_ring, DMA_TX_BD_CNT, pointer_bd);
	if (status != XST_SUCCESS) {
		XAxiDma_BdRingUnAlloc(tx_ring, DMA_TX_BD_CNT, pointer_bd);
		return status;
	}

	return XST_SUCCESS;
}


/**
 * @brief Handler of DMA RX interruption
 * @param[inout] context - global context (contains dma/ad/etc controllers)
 *
 * @return none
 */
void dma_rx_intr_handler(void* context) {
	dma_interrupt_handler_t* ctx = (dma_interrupt_handler_t*)context;
	XAxiDma_BdRing* rx_ring = XAxiDma_GetRxRing(ctx->dma->instance);

	uint32_t irq = XAxiDma_BdRingGetIrq(rx_ring);
	if (!(irq & XAXIDMA_IRQ_ALL_MASK))
		return;

	XAxiDma_BdRingAckIrq(rx_ring, irq);
    if (irq & XAXIDMA_IRQ_ERROR_MASK) {
    	uint32_t timeout = 10000;
		XAxiDma_Reset(ctx->dma->instance);
		while (timeout) {
			if (XAxiDma_ResetIsDone(ctx->dma->instance))
				break;
			timeout -= 1;
		}
    	return;
    }

	if (irq & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK)) {
		XAxiDma_Bd* pointer_bd = NULL;
		int processed_bd = XAxiDma_BdRingFromHw(rx_ring, XAXIDMA_ALL_BDS,
												&pointer_bd);
		if (processed_bd == 0)
			return;

		XAxiDma_BdRingIntDisable(rx_ring, XAXIDMA_IRQ_ALL_MASK);
		struct dma_pkt_t {
			uint8_t* buffer;
		    uint32_t size;
		    uint32_t completed;
		} rx_pkt = { .completed = 0, .size = 0 };

		XAxiDma_Bd* current_bd = pointer_bd;
		for (int i = 0; i < processed_bd; i++) {
			UINTPTR  buff_addr  = XAxiDma_BdGetBufAddr(current_bd);
			uint32_t recv_size  = XAxiDma_BdGetActualLength(current_bd,
													 ~XAXIDMA_BD_STS_ALL_MASK);
			uint32_t status_reg = XAxiDma_BdGetSts(current_bd);
			if (status_reg & XAXIDMA_BD_STS_COMPLETE_MASK) {
				if (status_reg & XAXIDMA_BD_STS_RXSOF_MASK) {
					rx_pkt.buffer = (uint8_t*)buff_addr;
					rx_pkt.size   = recv_size;
				} else {
					rx_pkt.size += recv_size;
				}

				if (status_reg & XAXIDMA_BD_STS_RXEOF_MASK) {
					rx_pkt.completed = 1;
				}
			}

			if (rx_pkt.completed) {
				Xil_DCacheInvalidateRange(buff_addr, rx_pkt.size);
				ctx->dma->callback(rx_pkt.buffer, rx_pkt.size, ctx->user_context);
				rx_pkt.completed = 0;
			}

			current_bd = (XAxiDma_Bd*)XAxiDma_BdRingNext(rx_ring, current_bd);
		}

		XAxiDma_BdRingFree(rx_ring, processed_bd, pointer_bd);
		int free_cnt_bd = rx_ring->FreeCnt;

		XAxiDma_BdRingAlloc(rx_ring, free_cnt_bd, &pointer_bd);
		XAxiDma_BdRingToHw(rx_ring, free_cnt_bd, pointer_bd);
		XAxiDma_BdRingIntEnable(rx_ring, XAXIDMA_IRQ_ALL_MASK);
	}
}


/**
 * @brief Handler of DMA TX interruption
 * @param[inout] context - global context (contains dma/ad/etc controllers)
 *
 * @return none
 */
void dma_tx_intr_handler(void* context) {
	dma_interrupt_handler_t* ctx = (dma_interrupt_handler_t*)context;
	XAxiDma_BdRing* tx_ring = XAxiDma_GetTxRing(ctx->dma->instance);

	uint32_t irq = XAxiDma_BdRingGetIrq(tx_ring);
	XAxiDma_BdRingAckIrq(tx_ring, irq);

	if (!(irq & XAXIDMA_IRQ_ALL_MASK))
		return;

	if (irq & XAXIDMA_IRQ_ERROR_MASK) {
		uint32_t timeout = 10000;
		XAxiDma_Reset(ctx->dma->instance);
		while (timeout) {
			if (XAxiDma_ResetIsDone(ctx->dma->instance))
				break;
			timeout -= 1;
		}
	}

	if (irq & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK)) {
		XAxiDma_BdRingIntDisable(tx_ring, XAXIDMA_IRQ_ALL_MASK);

		XAxiDma_Bd* pointer_bd = NULL;
		int processed_bd = XAxiDma_BdRingFromHw(tx_ring, XAXIDMA_ALL_BDS,
											    &pointer_bd);

		XAxiDma_BdRingFree(tx_ring, processed_bd, pointer_bd);
		XAxiDma_BdRingIntEnable(tx_ring, XAXIDMA_IRQ_ALL_MASK);
	}
}
