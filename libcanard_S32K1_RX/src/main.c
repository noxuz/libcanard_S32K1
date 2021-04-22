/*
 * Copyright (c) 2020, NXP. All rights reserved.
 * Distributed under The MIT License.
 * Author: Abraham Rodriguez <abraham.rodriguez@nxp.com>
 *
 * Description:
 * Example of using libcanard with UAVCAN V1.0 in the S32K1 platform
 * please reference the S32K1 manual, focused for development with
 * the UCANS32K146 board.
 * The files that are particular to this demo application are in the \src folder rather
 * than in the \include, where are the general libraries and headers.
 *
 * Transmits an UAVCAN Heartbeat message between two UCANS32K146 boards.
 *
 */

#include "uavcan\node\Heartbeat_1_0.h"
#include "libcanard\canard.h"
#include "media\canfd.h"
#include "o1heap\o1heap.h"
#include "timer\LPIT.h"
#include "clocks\SCG.h"
#include "S32K146_bitfields.h"


#define FLEXCAN_RX_IRQ_PRIO   	 (1u)

// Linker file symbols for o1heap allcator
extern int __HeapBase[];
extern int HEAP_SIZE[];

// allocator and instance declaration for wrappers
O1HeapInstance* my_allocator;
CanardInstance ins;

// Wrappers for using o1heap allocator with libcanard
static void* memAllocate(CanardInstance* const ins, const size_t amount);
static void memFree(CanardInstance* const ins, void* const pointer);

// Application-specific function prototypes
void FlexCAN0_reception_callback(void);
void abort(void);
void UCANS32K146_PIN_MUX();
void greenLED_init(void);
void greenLED_toggle(void);

// Global heartbeat message
volatile uavcan_node_Heartbeat_1_0 test_heartbeat;

// Frame for reception from the media layer from inside the FlexCAN callback
volatile fdframe_t frame_media;

// Message buffer used for receibing the CAN frames
const uint32_t RX_messageBuffer_index = 0;

static inline void S32_NVIC_DisableIRQ(IRQn_Type IRQn)
{
  S32_NVIC->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));

}

// LUT for converting form DLC to byte length in bytes
const uint8_t FlexCANDLC2Length[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Declarations for accessing the FlexCAN0 reception message buffer by the callback
typedef struct
{
    struct
	{
      __IOM uint32_t TIMESTAMP  : 16;
      __IOM uint32_t DLC        : 4;
      __IOM uint32_t RTR        : 1;
      __IOM uint32_t IDE        : 1;
      __IOM uint32_t SRR        : 1;
             uint32_t            : 1;
      __IOM uint32_t CODE       : 4;
             uint32_t            : 1;
      __IOM uint32_t ESI        : 1;
      __IOM uint32_t BRS        : 1;
      __IOM uint32_t EDL        : 1;
      __IOM uint32_t EXT_ID     : 29;
      __IOM uint32_t PRIO       : 3;
      __IOM uint32_t payload[16];
    } FD_MessageBuffer[7];
} CAN_MBUF_t;

#define CAN0_MBUF ((CAN_MBUF_t*)(CAN0_BASE + 0x80))
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

// Macro for swapping from little-endian to big- endian a 32-bit word
#if defined (__GNUC__) || defined (__ICCARM__) || defined (__ghs__) || defined (__ARMCC_VERSION)
#define S32_REV_BYTES(a, b) __asm volatile ("rev %0, %1" : "=r" (b) : "r" (a))
#endif

int main(void) {

	// Initialization of o1heap allocator for libcanard, requires 16-byte alignment, view linker file
	my_allocator = o1heapInit((void*)__HeapBase, (size_t)HEAP_SIZE, NULL, NULL);

	// Initialization of a canard instance with the previous allocator
	ins = canardInit(&memAllocate, &memFree);
	ins.mtu_bytes = CANARD_MTU_CAN_FD;
	ins.node_id = 97;

	// Subscribe to heartbeat messages within libCanard
	CanardRxSubscription heartbeat_subscription;

	(void) canardRxSubscribe(&ins,
							 CanardTransferKindMessage,
							 uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
							 uavcan_node_Heartbeat_1_0_EXTENT_BYTES_,
							 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
							 &heartbeat_subscription);

	/* Configure clock source */
	SCG_SOSC_8MHz_Init();
	SCG_SPLL_160MHz_Init();
	SCG_Normal_RUN_Init();

	// Indicative board LED for successful transmission
	greenLED_init();

	// 64-bit monotonic timer start
	LPIT0_Timestamping_Timer_Init();

	// Pin mux
	UCANS32K146_PIN_MUX();

	// Initialize FlexCAN0
	FlexCAN0_Init(CANFD_1MB_4MB_PLL, FLEXCAN_RX_IRQ_PRIO, FlexCAN0_reception_callback);

	// Disable interrupt since reception is done via polling
	S32_NVIC_DisableIRQ(CAN0_ORed_0_15_MB_IRQn );

	// Configure reception filters for the heartbeat message, message buffer 0
	FlexCAN0_Install_ID(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
						RX_messageBuffer_index);

	// Block waiting for reception interrupts to happen
	for(;;)
	{

	}

	return 0;
}


static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    return o1heapAllocate(my_allocator, amount);
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    o1heapFree(my_allocator, pointer);
}


void FlexCAN0_reception_callback(void)
{
		// Harvest the received frame from FlexCAN, verify that the message buffer 0 actually received
		if(CAN0->CAN0_IFLAG1_b.BUF0I)
		{
			// Proceed to read the rest of the fields
		    frame_media.EXTENDED_ID = CAN0_MBUF->FD_MessageBuffer[0].EXT_ID;
			frame_media.PAYLOAD_SIZE_BYTES = FlexCANDLC2Length[CAN0_MBUF->FD_MessageBuffer[0].DLC];

			// Leverage 32-bit native transfers
	        for (uint8_t i = 0;
	        		i < (frame_media.PAYLOAD_SIZE_BYTES >> 2); i++)
	        {
	            S32_REV_BYTES(CAN0_MBUF->FD_MessageBuffer[0].payload[i], frame_media.PAYLOAD[i]);
	        }


		// Convert from media frame to canard frame
		CanardFrame received_frame;
		received_frame.extended_can_id = frame_media.EXTENDED_ID;
		received_frame.payload_size = frame_media.PAYLOAD_SIZE_BYTES;
		received_frame.timestamp_usec = LPIT0_GetTimestamp();
		received_frame.payload = (void*)&frame_media.PAYLOAD;

		// Receive the transfer within libCanard
		CanardTransfer transfer;
		const int8_t res1 = canardRxAccept(&ins,
											 &received_frame,
											 0,
											 &transfer);

		if(res1 < 0){ abort(); } // Error occurred

		else if(res1 == 1) // A transfer was completed
		{
			// Instantiate a heartbeat message
			uavcan_node_Heartbeat_1_0 RX_hbeat;
			size_t hbeat_ser_buf_size = uavcan_node_Heartbeat_1_0_EXTENT_BYTES_;

			// De-serialize the heartbeat message
			int8_t res2 = uavcan_node_Heartbeat_1_0_deserialize_(&RX_hbeat, transfer.payload, &hbeat_ser_buf_size);

			if(res2 < 0){ abort(); } // Error occurred

			// Update global hbeat message
			test_heartbeat.uptime = RX_hbeat.uptime;
			test_heartbeat.health = RX_hbeat.health;
			test_heartbeat.mode = RX_hbeat.mode;

			// Deallocation of memory
			ins.memory_free(&ins, (void*)transfer.payload);

			// Toggle LED for successful reception
			greenLED_toggle();

		}
		else
		{
			// The received frame is not the last from a multi-frame transfer
		}

        // Unlock the mailbox by reading the free running timer
		(void)CAN0->CAN0_TIMER;
		// Clear the IFLAG1 flag (W1C)
		CAN0->CAN0_IFLAG1_b.BUF0I = 1;

		}
}

void abort(void)
{
	while(1){}
}


void UCANS32K146_PIN_MUX(void)
{
	/* Multiplex FlexCAN0 pins */
    PCC->PCC_PORTE_b.CGC = PCC_PCC_PORTE_CGC_1;   /* Clock gating to PORT E */
    PORTE->PORTE_PCR4_b.MUX = PORTE_PCR4_MUX_101; /* CAN0_RX at PORT E pin 4 */
    PORTE->PORTE_PCR5_b.MUX = PORTE_PCR5_MUX_101; /* CAN0_TX at PORT E pin 5 */

    PCC->PCC_PORTA_b.CGC = PCC_PCC_PORTA_CGC_1;   /* Clock gating to PORT A */
    PORTA->PORTA_PCR12_b.MUX = PORTA_PCR12_MUX_011; /* CAN1_RX at PORT A pin 12 */
    PORTA->PORTA_PCR13_b.MUX = PORTA_PCR13_MUX_011; /* CAN1_TX at PORT A pin 13 */

    /* Set to LOW the standby (STB) pin in both transceivers of the UCANS32K146 node board */
    PORTE->PORTE_PCR11_b.MUX = PORTE_PCR11_MUX_001; /* MUX to GPIO */
    PTE->GPIOE_PDDR |= 1 << 11; 				  /* Set direction as output */
    PTE->GPIOE_PCOR |= 1 << 11; 				  /* Set the pin LOW */

    PORTE->PORTE_PCR10_b.MUX = PORTE_PCR10_MUX_001; /* Same as above */
    PTE->GPIOE_PDDR |= 1 << 10;
    PTE->GPIOE_PCOR |= 1 << 10;
}

void greenLED_init(void)
{
	PCC->PCC_PORTD_b.CGC = PCC_PCC_PORTD_CGC_1; 	/* Enable clock for PORTD */
	PORTD->PORTD_PCR16_b.MUX = PORTE_PCR16_MUX_001; /* Port D16: MUX = GPIO */
    PTD->GPIOD_PDDR |= 1<<16;                       /* Port D16: Data direction = output  */

}

void greenLED_toggle(void)
{
	PTD->GPIOD_PTOR |= 1<<16;
}

