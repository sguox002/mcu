### SPI full duplex IT/DMA using LL driver

#### Application:
CW sampling at 80KHz to read two 16 bit data from two CW A/D per sample.
The A/D needs to get a readout command from master so it can send out the a/d value.
The A/D is connected to MCU SPI1.

#### SPI full duplex configuration:
spi full duplex needs 4 lines:
NSS for frame (software management)
SCK for clock
MOSI for data output
MISO for data input

full duplex: write occurs in MOSI and read occurs on MISO. each clock will latch a bit.
HAL_TransmitReceive
HAL_TransmitReceive_IT
HAL_TransmitReceive_DMA
and callback shall be:

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)

clock poloarity and phase:
00: clock idle at low and sampling at rising edge
11: clock idle at high and sampling at falling edge
the spi peripheral AD requires 00 configuration.

Cubemx use edge 1 for rising edge and 2 for falling edge.

data width:
4/8/16 bit. for more than 16 bits, we need use software managed frame signal.

If configured as 16 bit, a frame will has 16 clocks.
At the end of 16 clocks, the read data is updated in the data register.

SPI tx/rx fifo size is 4 bytes.
default when data width is 8 bits, fifo threshold is default 1 bytes, ie, when there is >=1 byte space for transmission, the TXE bit is set.
when rx fifo has >=1bytes received, RXNE bit will be set.

when data width is 16 bits, fifo threshold is default 2 bytes.

#### HAL vs LL.
HAL is not built on LL. HAL generally builds a handle with configuration and setting in its handle structure. HAL is less efficient.
LL uses macros to set/get registers and is more efficient.
HAL is more convenient.
HAL and LL can be mixed.
HAL already defined interrupt callbacks but LL not, so user has to define the ISR and callback themselves.

How to mix LL and HAL for the same peripheral:
- using cubemx generate HAL driver for spi. save the code for spi_init and Spi_MSP_init
- using cubemx generate LL driver.
- Then combine the two codes.
	- In init, you can first init LL and then HAL, note some values in LL would be overwritten, if the configuration is different.
	- So when we use LL, some relevent parameters shall be set again.

#### IT vs DMA
when using interrupt, these parameters are relevant:
- data width
- fifo threshold
- allows TXE and RXNE interrupt.
- SPI event handler and callbacks
- nss PA4 for the frame is managed by the software.
- one frame includes two samples and therefore 32 clocks.

16 bit data width, and so fifo threshold would be two bytes. Each sample then will generate two RXBE events.
So in each interrupt we read/write two bytes only.
Read/Write will automatically clear the TXE/RXNE flag.
TXE/RXNE is automatically set by hardware once they meet the fifo threshold.

```cpp
//timer interrupt...
	  LL_SPI_EnableIT_RXNE(SPI1);
	  /* Enable TXE   Interrupt             */
	  LL_SPI_EnableIT_TXE(SPI1);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //set nss low, AD_CONVST will become low

void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
	  /* Check RXNE flag value in ISR register */
	  if(LL_SPI_IsActiveFlag_RXNE(SPI1))
	  {
	    /* Call function Slave Reception Callback */
	    SPI1_Rx_Callback();
	  }
	  /* Check RXNE flag value in ISR register */
	  else if(LL_SPI_IsActiveFlag_TXE(SPI1))
	  {
	    /* Call function Slave Reception Callback */
	    SPI1_Tx_Callback();
	  }
	  /* Check STOP flag value in ISR register */
	  else if(LL_SPI_IsActiveFlag_OVR(SPI1))
	  {
	    /* Call Error function */
	    SPI1_TransferError_Callback();
	  }
  /* USER CODE END SPI1_IRQn 0 */
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

void  SPI1_Rx_Callback(void)
{
  /* Read character in Data register.
  RXNE flag is cleared by reading data in DR register */
	cwrx_buf[cw_dmaWritePointer++] = LL_SPI_ReceiveData16(SPI1);
	cw_bytes_recv+=2;
	cw_rx_byte-=2;
	if(cw_dmaWritePointer>=1024) cw_dmaWritePointer=0;
	if(cw_rx_byte==0){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //done with DMA transfer, set it high
		cw_rx_byte=4;
		cw_samples--;
		if(cw_samples==0) {
			HAL_TIM_Base_Stop_IT(&htim3);
			LL_SPI_Disable(SPI1);
		}
	}
}

void  SPI1_Tx_Callback(void)
{
  /* Write character in Data register.
  TXE flag is cleared by reading data in DR register */
  LL_SPI_TransmitData16(SPI1, cwtx_buf[cw_tx_ind++]);

  if(cw_tx_ind==2){
	  LL_SPI_DisableIT_TXE(SPI1);
	  cw_tx_ind=0;
  }
}

```

#### DMA
Interrupt still needs CPU to read data register to memory, which uses the bus and there exists idle time before transmission and after receiving.
This is due to LL_SPI_TransmitData16 and LL_SPI_ReceiveData16.

To understand the bus, cpu, peripherals, refer the MCU's block diagram.
Our MCU is stm32l476RG running at 40MHz.
SPI1 peripheral memory goes to MCU memory:
SPI1 goes to APB2 first, then goes to AHB1 and then goes to AHB and MCU read and store it in memory.

The memory transfer needs the MCU involving and the max frequency is barely 80KHz. (we have to do some fine tuning to reach this)
In our timer ISR, we first enable the TXE and RXNE event before we pull down the NSS, so we can give more time for transfer.

use DMA will be more efficient since the memory transfer needs no CPU involving.
SPI1 tx/rx can each have a DMA channel.

How does DMA work roughly?
similarly TXE and RXNE will still generate events.
DMA will start transfer:
- SPI1 enable DMA_TX/RX request
- TXE or RXNE event occurs.
- dma channel is enabled.

a dma transfer needs to define memory address and data length (note it is defined as the data width unit)
for our case, our data width is 16 bits, and each dma transfer is 4 bytes, then data length shall be 2 (int16)

Once the dma channel is enabled, it waits for the event and start to transfer.
we need to enable transfer completed event.

To restart a DMA transfer:
- make sure the fifo does not have unread data.
- disable the dma channel
- reconfigure the address and data length
- enable the SPI_dmaTx/rx requests.
- enable the dma transfer.

```cpp
in the timer interrupt:

	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //set nss low, AD_CONVST will become low
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);//transfer complete
	  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);//transfer error
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
	  LL_SPI_EnableDMAReq_RX(SPI1); //this is needed
	  /* Enable DMA TX Interrupt */
	  LL_SPI_EnableDMAReq_TX(SPI1);

	  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
	  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	  if(LL_DMA_IsActiveFlag_TC2(DMA1))
	  {
	    LL_DMA_ClearFlag_GI2(DMA1);
	    /* Call function Reception complete Callback */
	    DMA1_ReceiveComplete_Callback();
	  }
	  else if(LL_DMA_IsActiveFlag_TE2(DMA1))
	  {
	    /* Call Error function */
	    SPI1_DMA_TransferError_Callback();
	  }
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
	  if(LL_DMA_IsActiveFlag_TC3(DMA1))
	  {
	    LL_DMA_ClearFlag_GI3(DMA1);
	    /* Call function Transmission complete Callback */
	    DMA1_TransmitComplete_Callback();
	  }
	  else if(LL_DMA_IsActiveFlag_TE3(DMA1))
	  {
	    /* Call Error function */
	    SPI1_DMA_TransferError_Callback();
	  }
  /* USER CODE END DMA1_Channel3_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

void DMA1_ReceiveComplete_Callback(void)
{
  /* DMA Rx transfer completed */
	//LL_SPI_DisableDMAReq_RX(SPI1);//spix_cr2 rxdmaen, when this bit is set, it generates a dma request whenever rxne is set or not
	cw_bytes_recv+=4;
	cw_dmaWritePointer+=4;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //done with DMA transfer, set it high
	cw_samples--;
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

	if(cw_dmaWritePointer>=2048) {
		cw_dmaWritePointer=0;
		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
		LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)cwrx_buf,
		                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
	}
	else{
		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
		LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)(cwrx_buf+cw_dmaWritePointer),
		                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
	}

	if(cw_samples==0) {
		HAL_TIM_Base_Stop_IT(&htim3);
		LL_SPI_Disable(SPI1);
	}
}

/**
  * @brief  Function called from DMA1 IRQ Handler when Tx transfer is completed
  * @param  None
  * @retval None
  */
void DMA1_TransmitComplete_Callback(void)
{
	/* DMA Tx transfer completed */
	//LL_SPI_DisableDMAReq_TX(SPI1);//spix_cr2: dmatxen
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);//set ccr register
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)cwtx_buf, LL_SPI_DMA_GetRegAddr(SPI1),
	                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 2);
}
```

- using HAL, we can only reach 30KHz sampling frequency.
- using LL with IT, we can reach 80KHz barely.
- using LL with DMA, we can reach about 120KHz.




