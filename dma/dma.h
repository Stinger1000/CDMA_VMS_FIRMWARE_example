#ifndef SRC_DMA_N_H_
#define SRC_DMA_N_H_

#include "xaxidma.h"
#include "xscugic.h"

#define DMA_BD_ALIGN 	 	 XAXIDMA_BD_MINIMUM_ALIGNMENT
#define DMA_ETH_BD_BUFF_SIZE 2048 // size of data for one descriptor
#define DMA_PL_BD_BUFF_SIZE  2048

#define DMA_ETH_BD_CNT 512
#define DMA_PL_BD_CNT  512
#define DMA_TX_BD_CNT  1

#define DMA_ETH_BUFF_SIZE (DMA_ETH_BD_BUFF_SIZE * DMA_ETH_BD_CNT)
#define DMA_PL_BUFF_SIZE  (DMA_PL_BD_BUFF_SIZE  * DMA_PL_BD_CNT)

// == 4096
#define ETH_BD_SPACE_BYTES (XAxiDma_BdRingMemCalc(DMA_BD_ALIGN, DMA_ETH_BD_CNT))
#define PL_BD_SPACE_BYTES  (XAxiDma_BdRingMemCalc(DMA_BD_ALIGN, DMA_PL_BD_CNT))


typedef struct dma_dev {
	uint16_t device_id;

	uint8_t  rx_interrupt_priority;
	uint8_t  tx_interrupt_priority;
	uint32_t interrupt_rx_vec_id;
	uint32_t interrupt_tx_vec_id;

	uint32_t descriptor_size;	   // DMA_BD_ALIGN
	uint32_t descriptor_buff_size; // _BD_BUFF_SIZE
	uint32_t descriptors_count;	   // BD_CNT

	void* rx_descriptors_addr;     // segment of memory of descriptors
	void* tx_descriptors_addr;

	uint8_t* rx_buff_addr;         // for common rx buffer of data
	uint8_t* tx_buff_addr;         // for common tx buffer of data
} dma_dev_t;

typedef void (*dma_intr_callback_t)(uint8_t*, uint32_t, void*);

typedef struct dma {
	dma_dev_t 		    params;
    XAxiDma*	 		instance;
    XAxiDma_Config* 	config;
    dma_intr_callback_t callback;
    uint32_t 			tx_buffer_idx;
} dma_t;

typedef struct dma_interrupt_handler {
    dma_t* dma;
    void*  user_context;
} dma_interrupt_handler_t;


int dma_init(dma_t*);
int dma_setup_interrupts(dma_t*, XScuGic*, dma_interrupt_handler_t*);
int dma_transmit(dma_t*, const uint8_t*, uint32_t);

#endif
