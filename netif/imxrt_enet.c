/**
 * 	@brief picotcp enet driver for imxrt1050
 *
 */


#include "fsl_phy.h"
#include "fsl_enet.h"
#include "pico_device.h"
#include "app/dell_sm_tb.h"

/* number of enet descriptors */
#define ENET_RXBD_NUM				5
#define ENET_TXBD_NUM				3

/* buffer ssizes */
#define ENET_RXBUFF_SIZE 			(ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE 			(ENET_FRAME_MAX_FRAMELEN)

/* enet buffer alignement constraint */
#define FSL_ENET_BUFF_ALIGNMENT 	ENET_BUFF_ALIGNMENT

/* autonegotiation time */
#define ENET_ATONEGOTIATION_TIMEOUT 4095


/* pico device thread parameters */
#define NET_SPAWN_THREAD_PRIO		configMAX_PRIORITIES - 16
#define NET_SPAWN_THREAD_STK_SIZE	256



/* net buf aligned type macro */
typedef uint8_t rx_buffer_t[SDK_SIZEALIGN(ENET_RXBUFF_SIZE, FSL_ENET_BUFF_ALIGNMENT)];
typedef uint8_t tx_buffer_t[SDK_SIZEALIGN(ENET_TXBUFF_SIZE, FSL_ENET_BUFF_ALIGNMENT)];


/** internal variables */
AT_NONCACHEABLE_SECTION_ALIGN(static enet_rx_bd_struct_t rx_desc[ENET_RXBD_NUM], FSL_ENET_BUFF_ALIGNMENT);
AT_NONCACHEABLE_SECTION_ALIGN(static enet_tx_bd_struct_t tx_desc[ENET_TXBD_NUM], FSL_ENET_BUFF_ALIGNMENT);
SDK_ALIGN(static rx_buffer_t rxnetbuf[ENET_RXBD_NUM], FSL_ENET_BUFF_ALIGNMENT);
SDK_ALIGN(static tx_buffer_t txnetbuf[ENET_TXBD_NUM], FSL_ENET_BUFF_ALIGNMENT);

static enet_handle_t net_handle;
static SemaphoreHandle_t net_activity_sema;
static TimerHandle_t net_timer;
volatile unsigned int pico_ms_tick = 0;
bool net_ready = false;

/**
 * @fn ethernet_callback()
 * @brief IRQ handler of ethernet hardware
 */
static void ethernet_callback(ENET_Type *base, enet_handle_t *handle, enet_event_t event, uint8_t channel, void *param)
{
    struct pico_device *netif = (struct pico_device *)param;
    enet_interrupt_enable_t intflags = ENET_GetInterruptStatus(base);
    portBASE_TYPE ctw = pdFALSE;

    switch (event) {

        case kENET_RxEvent:
        	if(intflags & kENET_RxFrameInterrupt)
        		netif->__serving_interrupt = 1;
        		xSemaphoreGiveFromISR(net_activity_sema, &ctw);
            break;

        case kENET_TxEvent:

        break;
			if(intflags & kENET_RxFrameInterrupt)
				xSemaphoreGiveFromISR(net_activity_sema, &ctw);
        default:
            break;
    }

    portYIELD_FROM_ISR(ctw);
}




/**
 * @fn imxrt_pico_eth_send()
 * @brief output a packet through the ethernet interface
 *
 */
static int imxrt_pico_eth_send(struct pico_device *dev, void *buf, int len)
{
    status_t result;

    do {
    	/* try to push the buffer until the a tx buffer become ok,
    	 * this operation acts as a spin-lock preventing
    	 * issues with concurrent acces without the performance overhead
    	 * caused if a rtos mutex is used
    	 */
    	result = ENET_SendFrame(ENET, &net_handle, (const uint8_t *)buf, len);
    }while(result == kStatus_ENET_TxFrameBusy);

    /* data pushed, wait the ENET driver process it */
    xSemaphoreTake(net_activity_sema, portMAX_DELAY);


    return len;
}




/**
 * @fn imxrt_pico_eth_dsr()
 * @brief pushes packets from enet hw into the stack
 *
 */
static int imxrt_pico_eth_dsr(struct pico_device *dev, int loop_score)
{
    uint8_t *buf = NULL;
    uint32_t len = 0;

    ENET_GetRxFrameSize(&net_handle, &len);
    if(len) {
    	/* seems it is a valid frame, obtain it */
    	if(ENET_ReadFrame(ENET, &net_handle, buf, len) == kStatus_Success){
    		pico_stack_recv(dev, buf, len);
    		loop_score--;

    	}
    }

    /* clear the isr serving flag */
    __disable_irq();
    dev->__serving_interrupt = 0;
    __enable_irq();

    return loop_score;
}

/**
 * @fn imxrt_pico_net_timer_cb()
 * @brief called at every tick, requests stack to run
 */

static void imxrt_pico_tmr_callback(void *unused)
{
	(void)unused;

	/** TODO: this is a weird approach to implement
	 * the daemons of ipstack here in driver layer
	 * but for initial development seems a low
	 * risk approach, but in a future we need
	 * to consider move both timer cb and spawn threads
	 * to the arch folder
	 */


	/* update the tick, and clock the process
	 * of ipstack
	 */
	if(net_ready) {
		pico_ms_tick++;
		xSemaphoreGive(net_activity_sema);
	}
}


/**
 * @fn imxrt_pico_spawn_thread()
 * @brief IP stack core thread
 *
 */
static void imxrt_pico_spawn_thread(void *arg)
{
	/** TODO: it is a weird approach implement
	 * the daemons of ipstack here in driver layer
	 * but for initial development seems a low
	 * risk approach, but in a future we need
	 * to consider move both timer cb and spawn threads
	 * to the arch folder
	 */

	(void)arg;

	global_synchronize();
	net_ready = true;
	xTimerStart(net_timer, 0);



	for(;;) {
		/* wait for tx, rx or time trigger
		 * network activity
		 */
		xSemaphoreTake(net_activity_sema, portMAX_DELAY);
		pico_stack_tick();
		pico_bsd_stack_tick();
	}

}


struct pico_device *pico_eth_create(const char *name, const uint8_t *mac)
{
    /* Create device struct */
    enet_config_t config;
    bool link = false;
    uint32_t count = 0;
    enet_buffer_config_t buf_cfg;
	struct pico_device* eth_dev = PICO_ZALLOC(sizeof(struct pico_device));


    if(!eth_dev) {
        return NULL;
    }


    /*initalize the ethernet buffer descriptors */
    buf_cfg.rxBdNumber = ENET_RXBD_NUM;
    buf_cfg.txBdNumber = ENET_TXBD_NUM;
    buf_cfg.rxBuffSizeAlign = sizeof(rx_buffer_t);
    buf_cfg.txBuffSizeAlign = sizeof(tx_buffer_t);
    buf_cfg.rxBdStartAddrAlign = &rx_desc[0];
    buf_cfg.txBdStartAddrAlign = &tx_desc[0];
    buf_cfg.rxBufferAlign = &rxnetbuf[0][0];
    buf_cfg.txBufferAlign = &txnetbuf[0][0];


    /* create the auxiliary kernel objects */
    net_activity_sema = xSemaphoreCreateBinary();
    assert_tb(net_activity_sema != NULL);

    /* creates the net tick timer */
    net_timer = xTimerCreate("net_timer", 1, 1, NULL, (TimerCallbackFunction_t)imxrt_pico_tmr_callback);
    assert_tb(net_timer != NULL);

    BaseType_t err;
	err = xTaskCreate(imxrt_pico_spawn_thread, "net_device", NET_SPAWN_THREAD_STK_SIZE, NULL, NET_SPAWN_THREAD_PRIO, NULL);
	assert_tb(err == pdPASS);

    ENET_GetDefaultConfig(&config);


    /* initialize the enet external phy */
    PHY_Init(ENET, 0x02, SystemCoreClock);
    while ((count < ENET_ATONEGOTIATION_TIMEOUT) && (!link))
    {
        PHY_GetLinkStatus(ENET, 0x02, &link);

        if (link) {
        	break;
        }

        count++;
    }


    config.interrupt |= kENET_RxFrameInterrupt | kENET_TxFrameInterrupt | kENET_TxBufferInterrupt;

    /* Initialize the ENET module.*/
    ENET_Init(ENET, &net_handle, &config, &buf_cfg, mac, SystemCoreClock);
    NVIC_SetPriority(ENET_IRQn, 0xFF);
	NVIC_EnableIRQ(ENET_IRQn);


    ENET_SetCallback(&net_handle, (enet_callback_t)ethernet_callback, eth_dev);
    ENET_ActiveRead(ENET);



    /* Attach function pointers */
    eth_dev->send = imxrt_pico_eth_send;
    eth_dev->dsr = imxrt_pico_eth_dsr;
    eth_dev->poll = NULL;



    /* Register the device in picoTCP */
    if( 0 != pico_device_init(eth_dev, name, mac)) {
        dbg("Device init failed.\n");
        PICO_FREE(eth_dev);
        return NULL;
    }

    /* Return a pointer to the device struct */ 
    return eth_dev;
}




