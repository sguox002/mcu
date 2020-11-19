## STM32L476 USB driver</br>
References:</br>
stm32l4xx CubeMX driver code</br>
stm32l476 MCU reference manual</br>
stm32 USB Device Library manual</br>
USB complete: the developer's guide</br>
Free Device Monitoring Studio by HHD for protocol analyzer.</br>
</br>
### Introduction</br>
We are focusing on the USB device for communication applications.</br>
</br>
- usb host and device mode</br>
USB communication can only be initiated by host. </br>
Even if the mouse, keyboard et al HID devices, the USB device has to inform the host before sending data.</br>
</br>
- transfer mode:</br>
control transfer: predefined transfer in USB specification and has to be supported in all USB devices.</br>
This is used by ep0 in/out. control transfer can happen before address is assigned which is needed.</br>
bulk transfer: no timing requirement</br>
interrupt transfer: critical timing requirement</br>
isochronous transfer: streaming data, can tolerate loss of data.</br>
</br>
- device class</br>
class: combine the features for similar applications and define the common interface for these applications.</br>
common class such as CDC, HID.</br>
device class is built on top of the low level USB driver</br>
</br>
- To understand USB working, the stack structure shall be clear:</br>
the hardware: usb chip in the device and usb hub on the host. Transmitting and receiving are done by the hardware, </br>
and interrupt events are generated to drive the logic.</br>
device: the registers and interrupt generation, the ending point and buffers.</br>
host: usb device up and down.</br>
</br>
The low level driver on the device: interrupt service routing ISR and connecting with user code.</br>
including fulfill usb spec control transfer protocols via ep0 and user communications via other eps.</br>
</br>
device class driver: perform class defined protocols and callbacks.</br>
</br>
user code firmware: received data processing and connection with driver code.</br>
</br>
host (windows driver): </br>
</br>
host side applications: </br>
</br>
more about usb:</br>
</br>
USB device connected (plugged in): </br>
The hub is get notified by the presence of the USB device. The host begin the enumeration and set up the logic pipe from the host to device</br>
(the logic pipe is the communication path from host to device)</br>
Then the host will start send control transfer to get the device information and load the appropriate driver for the device.</br>
and assign the device address.</br>
Once driver is loaded, a session is established between the driver and the device. And now the usb is ready to use.</br>
</br>
application will use windows API to communicate with driver and device. The configuration such as baudrate, check comm state will involve a series of control transfer on the device class level protocol.</br>
</br>
usb communication uses data packet, the packet includes <pid, addr, ep#> and data. It needs a token packet for the start, and a ZLP (zero length packet) for the completion of transfer.)</br>
</br>
VCP connection:</br>
It will send 0x21/0x20 Set/Get_Line_Coding. This actually needs no changes if we are not querying the baudrate (since this is a virtual serial port)</br>
0x22: set control line state.</br>
</br>
CreateFile: just get the handle to the driver.</br>
GetCommState: Get_Line_Coding 0x21 control transfer.</br>
SetCommState: Set_Line_Coding 0x20 control transfer.</br>
EscapeCommFunction: 0x22 set control line status</br>
ClearCommError: only involves with host driver.</br>
PurgeComm: only involves with host driver</br>
FlushFileBuffers: only involves with host driver</br>
WriteFile: trigger a bulk transfer and data is sent back to host driver buffer.</br>
ReadFile: only involves with the driver.</br>
CloseHandle: only involve with the host driver.</br>
</br>
</br>
### OTG-FS (on the go full speed)</br>
OTG: the peripheral can switch device/host on the fly</br>
so it includes device stack and host stack.</br>
Here we only discuss the device stack.</br>
</br>
the usb device library is generic for all stm32 MCUs, only the HAL layer is adapted to each stm32 device.</br>
</br>
We only focus on the CDC (Communication Device) using VCP (Virtual comm port)</br>
VCP is from PSTN (public switched telephone network).</br>
</br>
some abbreviations:</br>
- FS/LS/HS: full speed, low speed, high speed</br>
- MAC: media access controller</br>
- OTG: On the go</br>
- PFC: packet FIFO controller</br>
- PHY: physical layer</br>
- ADP: Attach detection protocol</br>
- LPM: link power management</br>
- BCD: Battery charging detector</br>
- HNP: host negotiation protocol</br>
- SRP: session request protocol</br>
- PCD: Peripheral controller driver (Device mode)</br>
</br>
</br>
### Library architecture</br>
</br>
A generic USB communication stack:</br>
</br>
usb hardware<--></br>
HAL/LL usb device driver<--></br>
usb device class driver (CDC)<--></br>
User Firmware <--></br>
Windows Usb/VCP driver<--></br>
Windows USB Application.</br>
</br>
layer 1: core driver + class driver.</br>
core driver:</br>
- usb device core</br>
  APIs to manage internal state machine</br>
  callbacks to process interrupts</br>
- usb requests: </br>
- usb io requests: handle low level IO requests</br>
- log and debug</br>
</br>
Class driver:</br>
predefined class drivers which is linked to core driver using USBD_RegisterClass</br>
- CDC, HID, DFU...</br>
</br>
#### global data structure:</br>
- USBD_FS_DeviceDesc: device information;</br>
- FS_desc: the get descriptor callback functions</br>
- hUSBDeviceFS: usbd handle, including status, callbacks et al</br>
- USBD_Interface_fops_FS: CDC callbacks to connect user firmware.</br>
- USBD_CDC: cdc registered callbacks.</br>
- hpcd_USB_OTG_FS: handle for pcd device</br>
</br>
```</br>
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)</br>
  {</br>
    Error_Handler();</br>
  }</br>
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)</br>
  {</br>
    Error_Handler();</br>
  }</br>
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)</br>
  {</br>
    Error_Handler();</br>
  }</br>
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)</br>
  {</br>
    Error_Handler();</br>
  }</br>
</br>
</br>
```</br>
//FS_Desc defines the function pointers for the device descriptor</br>
```</br>
USBD_DescriptorsTypeDef FS_Desc =</br>
{</br>
  USBD_FS_DeviceDescriptor</br>
, USBD_FS_LangIDStrDescriptor</br>
, USBD_FS_ManufacturerStrDescriptor</br>
, USBD_FS_ProductStrDescriptor</br>
, USBD_FS_SerialStrDescriptor</br>
, USBD_FS_ConfigStrDescriptor</br>
, USBD_FS_InterfaceStrDescriptor</br>
#if (USBD_LPM_ENABLED == 1)</br>
, USBD_FS_USR_BOSDescriptor</br>
#endif /* (USBD_LPM_ENABLED == 1) */</br>
};</br>
```</br>
</br>
USBD_RegisterClass will:</br>
pdev->pClass=&USBD_CDC; //pClass pointer to the CDC driver.</br>
</br>
The CDC driver includes a set of function pointers (callback):</br>
</br>
```</br>
USBD_ClassTypeDef  USBD_CDC =</br>
{</br>
  USBD_CDC_Init,</br>
  USBD_CDC_DeInit,</br>
  USBD_CDC_Setup,</br>
  NULL,                 /* EP0_TxSent, */</br>
  USBD_CDC_EP0_RxReady,</br>
  USBD_CDC_DataIn,</br>
  USBD_CDC_DataOut,</br>
  NULL,</br>
  NULL,</br>
  NULL,</br>
  USBD_CDC_GetHSCfgDesc,</br>
  USBD_CDC_GetFSCfgDesc,</br>
  USBD_CDC_GetOtherSpeedCfgDesc,</br>
  USBD_CDC_GetDeviceQualifierDescriptor,</br>
};</br>
```</br>
USBD_CDC_RegisterInterface</br>
register the CDC interface callbacks</br>
pdev->pUserData=&USBD_Interface_fops_FS;</br>
```</br>
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =</br>
{</br>
  CDC_Init_FS,</br>
  CDC_DeInit_FS,</br>
  CDC_Control_FS,</br>
  CDC_Receive_FS</br>
};</br>
```</br>
These 4 callback functions shall be implemented by the user.</br>
Note: Transmit is not a callback.</br>
</br>
hUSBDeviceFS is the following data structure:</br>
```</br>
typedef struct _USBD_HandleTypeDef</br>
{</br>
  uint8_t                 id;</br>
  uint32_t                dev_config;</br>
  uint32_t                dev_default_config;</br>
  uint32_t                dev_config_status;</br>
  USBD_SpeedTypeDef       dev_speed;</br>
  USBD_EndpointTypeDef    ep_in[15]; //end point in </br>
  USBD_EndpointTypeDef    ep_out[15]; //end point out</br>
  uint32_t                ep0_state;</br>
  uint32_t                ep0_data_len;</br>
  uint8_t                 dev_state;</br>
  uint8_t                 dev_old_state;</br>
  uint8_t                 dev_address;</br>
  uint8_t                 dev_connection_status;</br>
  uint8_t                 dev_test_mode;</br>
  uint32_t                dev_remote_wakeup;</br>
</br>
  USBD_SetupReqTypedef    request; //setup request</br>
  USBD_DescriptorsTypeDef *pDesc; //descriptor callbacks point to FS_desc</br>
  USBD_ClassTypeDef       *pClass; //cdc class callbacks,point to USBD_CDC</br>
  void                    *pClassData; //pointer to USBD_CDC_HandleTypeDef</br>
  void                    *pUserData; //pointer to USBD_Interface_fops_FS (user interface)</br>
  void                    *pData;//pointer to the hpcd_USB_OTG_FS</br>
} USBD_HandleTypeDef;</br>
```</br>
pClassData stores the tx/rx status for CDC class:</br>
```</br>
typedef struct</br>
{</br>
  uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE / 4U];      /* Force 32bits alignment */</br>
  uint8_t  CmdOpCode;</br>
  uint8_t  CmdLength;</br>
  uint8_t  *RxBuffer;</br>
  uint8_t  *TxBuffer;</br>
  uint32_t RxLength;</br>
  uint32_t TxLength;</br>
</br>
  __IO uint32_t TxState;</br>
  __IO uint32_t RxState;</br>
}</br>
USBD_CDC_HandleTypeDef;</br>
```</br>
dynamically allocated memory buffer and updated using user provided cdc functions.</br>
for example SetTxBuffer, SetRxBuffer, Prepair_ReceivePacket et al.</br>
</br>
The hUSBDeviceFS is the core data structure which includes all necessary information.</br>
</br>
The hardware register status is in hpcd_USB_OTG_FS.instance.</br>
</br>
hUSBDeviceFS (USBD_HandleTypeDef)</br>
|_ep_in (USBD_EndpointTypeDef)</br>
|    |_status,is_used,total_len,rem_len,max_packet_size</br>
|_ep_out (USBD_EndpointTypeDef)</br>
|	|_status,is_used,total_len,rem_len,max_packet_size</br>
|_request (USBD_SetupReqTypedef)</br>
|	|_bmRequest, bRequest, wValue, wIndex, wLength</br>
|_pDesc=&Desc_FS (USBD_DescriptorsTypeDef)</br>
|_pClass=&USBD_CDC (USBD_ClassTypeDef)</br>
|       |_USBD_CDC_Init/DeInit</br>
|		|_USBD_CDC_Setup</br>
|		|_USBD_CDC_EP0_RxReady</br>
|		|_USBD_CDC_DataIn/Out</br>
|_pClassData=allocated memory (USBD_CDC_HandleTypeDef)</br>
|       |_buffer for a packet-</br>
|		|_rx buffer,tx buffer</br>
|		|_rx size and tx size</br>
|		|_rx state and tx state</br>
|_pUserData=&USBD_Interface_fops_FS (user cdc interface)</br>
|       |_CDC_Init/DeInit_FS</br>
|		|_CDC_Control_FS</br>
|		|_CDC_Receive_FS</br>
|_pData=&hpcd_USB_OTG_FS (__PCD_HandleTypeDef)</br>
|        |_Instance=&USB_basex (PCD_TypeDef, USB global registers, memory mapped)</br>
|		|_Init (PCD_InitTypeDef)</br>
|		|_USB_Address</br>
|		|_In_ep (PCD_EPTypeDef)</br>
|		    |_Ep_num, dir, is_stall,type</br>
|			|_pid_start, even_odd_frame, tx_fifo_num</br>
|			|_max_pack_size,xfer_buff,dma_addr,xfer_len,xfer_count</br>
|		|_Out_ep (PCD_EPTypeDef)</br>
|		|_Lock</br>
|		|_state</br>
|		|_error code</br>
|		|_Setup buffer</br>
|		|_LPM_sate,BESL,lpm_active,battery_charging_active</br>
|		|_pdata=&hUSBDeviceFS (pointer to upper stack handler)</br>
|		|_callback pointers</br>
		</br>
The process to initialize the hUsbDeviceFS:</br>
- usbd_init: set the descriptor and callbacks</br>
   |_set pClass to null.</br>
   |_set pDesc</br>
   |_default state</br>
   |_id</br>
   |_USBD_LL_Init: init the hpcd_USB_OTG_FS and link to hUSBDeviceFS</br>
		|_hpcd_USB_OTG_FS.pData and hUSBDeviceFS.pdata linked to each other</br>
		|_instance pointer to usb global register</br>
		|_init structure</br>
		|_HAL_PCD_Init: </br>
		|   |_HAL_PCD_MSPInit: enable clock, Pin configure, interrupt enable.</br>
		|	|_PCD_BUSY (no communication allowed)</br>
		|	|_Disable interrupts</br>
		|	|_USB_CoreInit (reset the usb core)</br>
		|	|_Init the in/out ep data structure</br>
		|	|_USB_DevInit: Init USB device mode registers</br>
		|	|_set address 0</br>
		|	|_PCD_Ready: now able to communicate with host</br>
		|	|_DevDisconnect</br>
		|_SetRxFifo, SetTxFifo 0 and SetTxFifo 1</br>
- usbd_RegisterClass: hang USBD_CDC to pClass.</br>
- usbd_CDC_RegisterInterface: hang USBD_Interface_fops_FS to pUser.</br>
- USBD_Start</br>
	|_USBD_LL_Start</br>
		|_HAL_PCD_Start: dynamically allocate pData</br>
			|_USB_DevConnect</br>
			|_HAL_PCD_Enable</br>
</br>
now the USB device is up and everything is handled by the PCD Interrupt Handler.</br>
The handler will drive the callback from low level to high level.</br>
	</br>
		</br>
## low level driver: files</br>
stm32l4xx_ll_usb.h/c lower level usb driver for the OTG.</br>
stm32l4xx_hal_pcd.h/c lower level driver for PCD (Device mode)</br>
</br>
stm32l4xx_ll_usb.h/c:</br>
```</br>
/* Initialization and de-initialization functions  ******************************/</br>
HAL_StatusTypeDef HAL_Init(void);</br>
HAL_StatusTypeDef HAL_DeInit(void);</br>
void HAL_MspInit(void);</br>
void HAL_MspDeInit(void);</br>
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);</br>
</br>
/* Peripheral Control functions  ************************************************/</br>
void HAL_IncTick(void);</br>
void HAL_Delay(uint32_t Delay);</br>
uint32_t HAL_GetTick(void);</br>
void HAL_SuspendTick(void);</br>
void HAL_ResumeTick(void);</br>
uint32_t HAL_GetHalVersion(void);</br>
uint32_t HAL_GetREVID(void);</br>
uint32_t HAL_GetDEVID(void);</br>
uint32_t HAL_GetUIDw0(void);</br>
uint32_t HAL_GetUIDw1(void);</br>
uint32_t HAL_GetUIDw2(void);</br>
</br>
/* DBGMCU Peripheral Control functions  *****************************************/</br>
void HAL_DBGMCU_EnableDBGSleepMode(void);</br>
void HAL_DBGMCU_DisableDBGSleepMode(void);</br>
void HAL_DBGMCU_EnableDBGStopMode(void);</br>
void HAL_DBGMCU_DisableDBGStopMode(void);</br>
void HAL_DBGMCU_EnableDBGStandbyMode(void);</br>
void HAL_DBGMCU_DisableDBGStandbyMode(void);</br>
</br>
/* SYSCFG Control functions  ****************************************************/</br>
void HAL_SYSCFG_SRAM2Erase(void);</br>
void HAL_SYSCFG_EnableMemorySwappingBank(void);</br>
void HAL_SYSCFG_DisableMemorySwappingBank(void);</br>
</br>
#if defined(VREFBUF)</br>
void HAL_SYSCFG_VREFBUF_VoltageScalingConfig(uint32_t VoltageScaling);</br>
void HAL_SYSCFG_VREFBUF_HighImpedanceConfig(uint32_t Mode);</br>
void HAL_SYSCFG_VREFBUF_TrimmingConfig(uint32_t TrimmingValue);</br>
HAL_StatusTypeDef HAL_SYSCFG_EnableVREFBUF(void);</br>
void HAL_SYSCFG_DisableVREFBUF(void);</br>
#endif /* VREFBUF */</br>
</br>
void HAL_SYSCFG_EnableIOAnalogSwitchBooster(void);</br>
void HAL_SYSCFG_DisableIOAnalogSwitchBooster(void);</br>
```</br>
define the states and error codes</br>
```</br>
/**</br>
  * @brief  USB Mode definition</br>
  */</br>
typedef enum</br>
{</br>
  USB_DEVICE_MODE  = 0,</br>
  USB_HOST_MODE    = 1,</br>
  USB_DRD_MODE     = 2</br>
} USB_ModeTypeDef;</br>
</br>
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)</br>
/**</br>
  * @brief  URB States definition</br>
  */</br>
typedef enum</br>
{</br>
  URB_IDLE = 0,</br>
  URB_DONE,</br>
  URB_NOTREADY,</br>
  URB_NYET,</br>
  URB_ERROR,</br>
  URB_STALL</br>
} USB_OTG_URBStateTypeDef;</br>
</br>
/**</br>
  * @brief  Host channel States  definition</br>
  */</br>
typedef enum</br>
{</br>
  HC_IDLE = 0,</br>
  HC_XFRC,</br>
  HC_HALTED,</br>
  HC_NAK,</br>
  HC_NYET,</br>
  HC_STALL,</br>
  HC_XACTERR,</br>
  HC_BBLERR,</br>
  HC_DATATGLERR</br>
} USB_OTG_HCStateTypeDef;</br>
</br>
```</br>
</br>
data structures:</br>
</br>
otg_cfg:</br>
```</br>
typedef struct</br>
{</br>
  uint32_t dev_endpoints;        /*!< Device Endpoints number.</br>
                                      This parameter depends on the used USB core.</br>
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 15 */</br>
</br>
  uint32_t Host_channels;        /*!< Host Channels number.</br>
                                      This parameter Depends on the used USB core.</br>
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 15 */</br>
</br>
  uint32_t speed;                /*!< USB Core speed.</br>
                                      This parameter can be any value of @ref USB_Core_Speed_                */</br>
</br>
  uint32_t dma_enable;           /*!< Enable or disable of the USB embedded DMA used only for OTG HS.                             */</br>
</br>
  uint32_t ep0_mps;              /*!< Set the Endpoint 0 Max Packet size.</br>
                                      This parameter can be any value of @ref USB_EP0_MPS_                   */</br>
</br>
  uint32_t phy_itface;           /*!< Select the used PHY interface.</br>
                                      This parameter can be any value of @ref USB_Core_PHY_                  */</br>
</br>
  uint32_t Sof_enable;           /*!< Enable or disable the output of the SOF signal.                        */</br>
</br>
  uint32_t low_power_enable;     /*!< Enable or disable the low power mode.                                  */</br>
</br>
  uint32_t lpm_enable;           /*!< Enable or disable Link Power Management.                               */</br>
</br>
  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                 */</br>
</br>
  uint32_t vbus_sensing_enable;  /*!< Enable or disable the VBUS Sensing feature.                            */</br>
</br>
  uint32_t use_dedicated_ep1;    /*!< Enable or disable the use of the dedicated EP1 interrupt.              */</br>
</br>
  uint32_t use_external_vbus;    /*!< Enable or disable the use of the external VBUS.                        */</br>
} USB_OTG_CfgTypeDef;</br>
```</br>
End point data structure:</br>
```</br>
typedef struct</br>
{</br>
  uint8_t   num;            /*!< Endpoint number</br>
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15    */</br>
</br>
  uint8_t   is_in;          /*!< Endpoint direction</br>
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */</br>
</br>
  uint8_t   is_stall;       /*!< Endpoint stall condition</br>
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */</br>
</br>
  uint8_t   type;           /*!< Endpoint type</br>
                                 This parameter can be any value of @ref USB_EP_Type_                     */</br>
</br>
  uint8_t   data_pid_start; /*!< Initial data PID</br>
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */</br>
</br>
  uint8_t   even_odd_frame; /*!< IFrame parity</br>
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 1    */</br>
</br>
  uint16_t  tx_fifo_num;    /*!< Transmission FIFO number</br>
                                 This parameter must be a number between Min_Data = 1 and Max_Data = 15   */</br>
</br>
  uint32_t  maxpacket;      /*!< Endpoint Max packet size</br>
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */</br>
</br>
  uint8_t   *xfer_buff;     /*!< Pointer to transfer buffer                                               */</br>
</br>
  uint32_t  dma_addr;       /*!< 32 bits aligned transfer buffer address                                  */</br>
</br>
  uint32_t  xfer_len;       /*!< Current transfer length                                                  */</br>
</br>
  uint32_t  xfer_count;     /*!< Partial transfer length in case of multi packet transfer                 */</br>
} USB_OTG_EPTypeDef;</br>
```</br>
</br>
stm32l4xx_hal_pcd.h/c</br>
the device mode (peripheral controller device) driver:</br>
</br>
state machine:</br>
```</br>
/**</br>
  * @brief  PCD State structure definition</br>
  */</br>
typedef enum</br>
{</br>
  HAL_PCD_STATE_RESET   = 0x00,</br>
  HAL_PCD_STATE_READY   = 0x01,</br>
  HAL_PCD_STATE_ERROR   = 0x02,</br>
  HAL_PCD_STATE_BUSY    = 0x03,</br>
  HAL_PCD_STATE_TIMEOUT = 0x04</br>
} PCD_StateTypeDef;</br>
</br>
/* Device LPM suspend state */</br>
typedef enum</br>
{</br>
  LPM_L0 = 0x00, /* on */</br>
  LPM_L1 = 0x01, /* LPM L1 sleep */</br>
  LPM_L2 = 0x02, /* suspend */</br>
  LPM_L3 = 0x03, /* off */</br>
} PCD_LPM_StateTypeDef;</br>
</br>
typedef enum</br>
{</br>
  PCD_LPM_L0_ACTIVE = 0x00, /* on */</br>
  PCD_LPM_L1_ACTIVE = 0x01, /* LPM L1 sleep */</br>
} PCD_LPM_MsgTypeDef;</br>
</br>
typedef enum</br>
{</br>
  PCD_BCD_ERROR                     = 0xFF,</br>
  PCD_BCD_CONTACT_DETECTION         = 0xFE,</br>
  PCD_BCD_STD_DOWNSTREAM_PORT       = 0xFD,</br>
  PCD_BCD_CHARGING_DOWNSTREAM_PORT  = 0xFC,</br>
  PCD_BCD_DEDICATED_CHARGING_PORT   = 0xFB,</br>
  PCD_BCD_DISCOVERY_COMPLETED       = 0x00,</br>
</br>
} PCD_BCD_MsgTypeDef;</br>
```</br>
</br>
__PCD_HandleTypeDef:</br>
```</br>
typedef struct __PCD_HandleTypeDef</br>
{</br>
  PCD_TypeDef             *Instance;   /*!< Register base address              */</br>
  PCD_InitTypeDef         Init;        /*!< PCD required parameters            */</br>
  __IO uint8_t            USB_Address; /*!< USB Address                        */</br>
  PCD_EPTypeDef           IN_ep[16];   /*!< IN endpoint parameters             */</br>
  PCD_EPTypeDef           OUT_ep[16];  /*!< OUT endpoint parameters            */</br>
  HAL_LockTypeDef         Lock;        /*!< PCD peripheral status              */</br>
  __IO PCD_StateTypeDef   State;       /*!< PCD communication state            */</br>
  __IO  uint32_t          ErrorCode;   /*!< PCD Error code                     */</br>
  uint32_t                Setup[12];   /*!< Setup packet buffer                */</br>
  PCD_LPM_StateTypeDef    LPM_State;   /*!< LPM State                          */</br>
  uint32_t                BESL;</br>
</br>
</br>
  uint32_t lpm_active;                 /*!< Enable or disable the Link Power Management .</br>
                                       This parameter can be set to ENABLE or DISABLE        */</br>
</br>
  uint32_t battery_charging_active;    /*!< Enable or disable Battery charging.</br>
                                       This parameter can be set to ENABLE or DISABLE        */</br>
  void                    *pData;      /*!< Pointer to upper stack Handler */</br>
    void (* SOFCallback)(struct __PCD_HandleTypeDef *hpcd);                              /*!< USB OTG PCD SOF callback                */</br>
  void (* SetupStageCallback)(struct __PCD_HandleTypeDef *hpcd);                       /*!< USB OTG PCD Setup Stage callback        */</br>
  void (* ResetCallback)(struct __PCD_HandleTypeDef *hpcd);                            /*!< USB OTG PCD Reset callback              */</br>
  void (* SuspendCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Suspend callback            */</br>
  void (* ResumeCallback)(struct __PCD_HandleTypeDef *hpcd);                           /*!< USB OTG PCD Resume callback             */</br>
  void (* ConnectCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Connect callback            */</br>
  void (* DisconnectCallback)(struct __PCD_HandleTypeDef *hpcd);                       /*!< USB OTG PCD Disconnect callback         */</br>
</br>
  void (* DataOutStageCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);      /*!< USB OTG PCD Data OUT Stage callback     */</br>
  void (* DataInStageCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);       /*!< USB OTG PCD Data IN Stage callback      */</br>
  void (* ISOOUTIncompleteCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);  /*!< USB OTG PCD ISO OUT Incomplete callback */</br>
  void (* ISOINIncompleteCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);   /*!< USB OTG PCD ISO IN Incomplete callback  */</br>
  void (* BCDCallback)(struct __PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);      /*!< USB OTG PCD BCD callback                */</br>
  void (* LPMCallback)(struct __PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);      /*!< USB OTG PCD LPM callback                */</br>
</br>
  void (* MspInitCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Msp Init callback           */</br>
  void (* MspDeInitCallback)(struct __PCD_HandleTypeDef *hpcd);        /*!< USB OTG PCD Msp DeInit callback         */</br>
  } PCD_HandleTypeDef;</br>
</br>
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);</br>
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);</br>
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);</br>
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);</br>
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);</br>
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);</br>
uint16_t          HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);</br>
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);  </br>
  ```</br>
</br>
#### program API</br>
### configure USB driver</br>
- device init: HAL_PCD_Init</br>
- End point configuration</br>
  HAL_PCD_EP_Open</br>
  HAL_PCD_EP_Close</br>
- device core--USBD_HandleTypeDef</br>
  the handle keeps all information in realtime as well as the state machine and endpoint information</br>
  EP0 state:</br>
	  0 idle</br>
	  1 setup</br>
	  2 data_in</br>
	  3 data_out</br>
	  4 status_in</br>
	  5 status_out</br>
	  6 stall</br>
  Device state:</br>
	  1 default</br>
	  2 addressed</br>
	  3 configured</br>
	  4 suspended</br>
	  note: the usb spec defines 6 states: attached, powered, default,addressed, configured, suspended.</br>
   when there is problem try to exam this first.</br>
- data transfer</br>
  HAL_PCD_EP_Transmit</br>
  HAL_PCD_EP_Receive</br>
  HAL_PCD_EP_SetStall</br>
  HAL_PCD_EP_ClrStall</br>
  HAL_PCD_EP_Flush</br>
  HAL_PCD_IRQHandler</br>
- handling control endpoint (EP0)</br>
  usb transfer: control, interrupt, bulk, isochronous</br>
  (isochronous: equal time interval, 等时的)</br>
  control packet: standard, class specific, vendor specific</br>
  control packet->setup packet.</br>
  EP0 receives the packet and process in interrupt and update the state machine.</br>
  setup packet are 8 bytes fixed.</br>
  ```</br>
typedef  struct  usb_setup_req</br>
{</br>
    uint8_t   bmRequest;</br>
    uint8_t   bRequest;</br>
    uint16_t  wValue;</br>
    uint16_t  wIndex;</br>
    uint16_t  wLength;</br>
}USBD_SetupReqTypedef;</br>
  ```</br>
  standard requests:</br>
  Get_Status: device/interface/ep0 out/ep0 in/ep.</br>
  Clear_Feature: clear the device remote wakeup feature/clear stall of EP. (not include EP0)</br>
  Set_Feature: opposite to above</br>
  Set_address: set device address</br>
  Get_Decriptor: device/config/string/</br>
  Get_configuration</br>
  set_configuration</br>
  get_interface</br>
  set_interface</br>
  </br>
  non standard requests:</br>
  all non standard requests are sent to class code to process</br>
  setup req ->pdev->pClass->Setup(pdev,req)</br>
  data stage: USBD_CtlSendData, USBD_CtlPrepareRx.</br>
  status stage   pdev->pClass->Setup(pdev,req)</br>
  </br>
- non control EP transfer</br>
  using in and out stage callback</br>
  interrupt mode</br>
  </br>
</br>
#### OTG registers  </br>
</br>
GOTGCTL: OTG global control register</br>
GOTGINT: OTG global interrupt register</br>
GAHBCFG: AHB config register</br>
GUSBCFG: USB config register</br>
GRSTCTL: reset control register</br>
GINTSTS: core interrupt status register</br>
GINTMSK: interrupt mask register</br>
GRXSTSR: receive status debug read register</br>
GRXSTSP: receive status debug pop register</br>
GRXFSIZ: receive FIFO size register</br>
GCCFG: general core config register</br>
CID: core ID register</br>
GLPMCFG: core LPM config register</br>
GPWRDN: powerdown register</br>
GADPCTL: adp tmer control/status register</br>
DIEPTXFx: device IN endpoint x=1 to 5</br>
</br>
Device mode CSR map (control/status register)</br>
DCFG: device config register</br>
DCTL: device control register</br>
DSTS: device status register</br>
DIEPMSK: device IN endpoint common interrupt mask register</br>
DOEPMSK: device OUT endpoint common interrupt mask reg</br>
DAINT: device all endpoint interrupt</br>
DAINTMSK: device all endpoint interrupt mask</br>
DVBUSDIS: device VBUS discharge timer reg</br>
DVBUSPULSE: device vbus pulsing timer reg.</br>
DIEPEMPMSK: device IN endpoint FIFO empty interrupt mask</br>
DIEPCTL0: device IN endpoint 0 control reg.</br>
DIEPCTLx: x=1:5</br>
DIEPINTx: device IN endpoint x interrupt reg. x=0-5</br>
DIEPTSIZ0: device IN endpoint 0 transfer size reg.</br>
DIEPTSIZx: x=1-5</br>
DOEPCTL0: device OUT endpoint 0 control reg.</br>
DOEPCTLx: x=1-5</br>
DOEPINTx</br>
DOEPTSIZ0:</br>
DOEPTSIZx:</br>
</br>
#### USB isr: HAL_PCD_IRQHandler</br>
The ISR is the heart of the USB device.</br>
It maintains the state machine and process the transfer completed events.</br>
The interrupt begins after the device is up.</br>
First handshaking including:</br>
enumeration</br>
get descriptor</br>
assign address</br>
configure the device</br>
bulk transfer.</br>
connect and disconnection</br>
</br>
device mode:</br>
- invalid: quit</br>
- MMIS: mode mismatch: clear</br>
- OEPINT:</br>
   - DAINT & DAINTMSK</br>
   - high 16 bit</br>
   - loop check all OUT ep</br>
     1. check if EPx intterupt is set</br>
	 2. DOEPINT & DOEPMSK</br>
	 3. if XFRC (transfer completed interrupt)</br>
	    clear XFRC</br>
		clear DOEPINT0 bit 15</br>
		clear DOEPINT0 bit 5</br>
		check STUP (setup phase done), clear DOEPINTx bit 15 and STUP</br>
		check OTEPDIS (OUT token received when endpoint disabled): clear it</br>
		DataOutStageCallback, this is where to interface to application.</br>
		</br>
- IEPINT:</br>
	- DAINT & DAINTMSK</br>
	- loop over all IN ep.</br>
		1. check if EPx interrupt</br>
		2. DIEPINT & DIEPMSK</br>
		3. check XFRC</br>
		   clear fifo empty mask</br>
		   clear xfrc bit</br>
		   DataInStageCallback (empty)</br>
		   check TOC (Timeout Condition), clear it</br>
		   check ITXFE (IN token received when Tx FIFO is empty) clear it</br>
		   check INEPNE (In token received with EP mistmatch)</br>
		   check EPDISD (EP disabled), clear it</br>
		   check TXFE (Transmit Fifo empty), clear it.</br>
- WKUINT (resume/remote wakeup detection interrupt)</br>
	- clear DCTL RWUSIG (Remote wakeup signalling),</br>
	- LPM callback</br>
	- claer WKUINT</br>
- USBSUSP (usb suspend): </br>
	- suspend callback</br>
	- clear flag</br>
- LPMINT: LPM interrupt</br>
- USBRST: USB reset</br>
	- clear RWUSIG</br>
	- flushTxFifo</br>
	- init INEP, OUTEP (Ctrl and INT, MSK</br>
	- set default address</br>
	- EP0 to receive setup packets USB_EP0_OutStart</br>
	- clear flag</br>
- ENUMDNE: Enumeration done.</br>
	- active setup</br>
	- init hpcd env.</br>
	- depending on HCLK, set the TRDT (turn around time) in USBCFG </br>
	- resetCallback</br>
	- clear flag</br>
- RXFLVL: Rx Fifo not empty</br>
	- mask RXFLVL</br>
	- get GRXSTSP (pop register)</br>
	- get ep num from pop reg.</br>
	- get PKTSTS (packet status) from pop reg. (0001, OUT NAK, 0010: out packet received, 0011: out transfer completed. 0100 setup completed. 0110 setup packet received)</br>
	  - 0010: STS_DATA_UPDT</br>
	    get BCNT (Bytes count 11 bits, max 2048 bytes) in the packet</br>
	    read packet to ep->xfer_buff</br>
	  - 0110: setup packet received</br>
	    read packet</br>
	- unmask.</br>
SOF: start of frame</br>
...</br>
IISOIXFR: Incomplete ISO IN transfer.</br>
...</br>
INCOMPISOOUT: Incomplete ISO OUT transfer.</br>
...</br>
SRQINT: session request/new session detected interrupt</br>
	- connection callback</br>
	- clear flag</br>
OTGINT: OTG interrupt, indicates an OTG protocol event.</br>
	- check GOTGINT:</br>
	- check SEDET bit (Session End detected)</br>
	- disconnection callback</br>
</br>
pay attention to:</br>
DataInStageCallback</br>
DataOutStageCallback</br>
SetupStageCallback</br>
</br>
setup stage:</br>
mcu driver must respond host's requests</br>
including</br>
- get descriptors (enumeration stage to find the driver for it).</br>
USBD_FS_DeviceDescriptor</br>
USBD_FS_LangIDStrDescriptor</br>
USBD_FS_ProductStrDescriptor</br>
USBD_FS_ManufacturerStrDescriptor</br>
USBD_FS_SerialStrDescriptor</br>
USBD_FS_ConfigStrDescriptor</br>
USBD_FS_InterfaceStrDescriptor</br>
USBD_FS_USR_BOSDescriptor</br>
Get_SerialNum</br>
</br>
- enumeration</br>
depending on the hclk used, it sets TRDT (turn around time)</br>
setup packets process has time limit requirement. 5 seconds.</br>
debugging with care.</br>
</br>
usbd_core.h/c: handle all usb communication and state machine</br>
usbd_req.h/c: request implementation in usb2.0 spec chapter 9 (not found, it is actually usbd_ctlreq.h/c)</br>
usbd_ctlreq.h/c: handle the results of usb transaction</br>
//usbd_conf_template.h/c: template file</br>
usbd_def.h/c: common definition for usb.</br>
these files are all for setup package process.</br>
</br>
usbd_cd.h/c used for data processing for non-ep0 data process.</br>
</br>
they are middlewares built on top of the lower level HAL drivers.</br>
</br>
## The stm32l476xx USB driver (PCD+CDC): </br>
</br>
low level driver</br>
|_stm32l4xx_ll_usb.h/c: low level usb register operations</br>
|_stm32l4xx_hal_pcd.h/c: HAL level PCD (peripheral mode) drivers. built on top of LL.</br>
|_stm32l4xx_hal_pcd_ex.h/c: extension module.</br>
</br>
dependency:</br>
|_ system_stm32l4xx.h.</br>
|_ stm32l4xx.h.</br>
|_ stm32l476xx,h.</br>
    |_ USB_OTG_GlobalTypeDef: bind registers and memory layouts.</br>
	|_ USB_OTG_DeviceTypeDef</br>
	|_ USB_OTG_INEndpointTypeDef</br>
	|_ USB_OTG_OUTEndpointTypeDef</br>
	|_ USB_OTG_HostTypeDef</br>
	|_ USB_OTG_HostChannelTypeDef</br>
</br>
middleware driver</br>
|_core</br>
|    |_usbd_core.h/c: all functions for usb transactions</br>
|    |_usbd_ctlreq.h/c: </br>
|    |_usbd_ioreq.h/c: </br>
|	 |_usbd_def.h</br>
|_class</br>
     |_usbd_cdc.h/c</br>
The middleware mainly:</br>
- process the usb protocol (high level setup packets)</br>
- provide cdc class - application interface.</br>
</br>
### stm32l4xx_ll_usb.h/c</br>
|_ USB_ModeTypeDef: define the USB mode: device, host, DRD</br>
|_ USB_OTG_URBStateTypeDef: URB state definition</br>
|_ USB_OTG_HCStateTypeDef: host channel state definition</br>
|_ USB_OTG_CfgTypeDef: USB OTG Initialization Structure definition</br>
|_ USB_OTG_EPTypeDef: usb endpoint data structure.</br>
|_ USB_OTG_HCTypeDef: usb otg host channel</br>
|_ register layout (defined in stm32l476xx.h)</br>
```</br>
#define USB_OTG_FS_PERIPH_BASE               (0x50000000UL)</br>
</br>
#define USB_OTG_GLOBAL_BASE                  (0x00000000UL)</br>
#define USB_OTG_DEVICE_BASE                  (0x00000800UL)</br>
#define USB_OTG_IN_ENDPOINT_BASE             (0x00000900UL)</br>
#define USB_OTG_OUT_ENDPOINT_BASE            (0x00000B00UL)</br>
#define USB_OTG_EP_REG_SIZE                  (0x00000020UL)</br>
#define USB_OTG_HOST_BASE                    (0x00000400UL)</br>
#define USB_OTG_HOST_PORT_BASE               (0x00000440UL)</br>
#define USB_OTG_HOST_CHANNEL_BASE            (0x00000500UL)</br>
#define USB_OTG_HOST_CHANNEL_SIZE            (0x00000020UL)</br>
#define USB_OTG_PCGCCTL_BASE                 (0x00000E00UL)</br>
#define USB_OTG_FIFO_BASE                    (0x00001000UL)</br>
#define USB_OTG_FIFO_SIZE                    (0x00001000UL)</br>
</br>
#define USBx_PCGCCTL    *(__IO uint32_t *)((uint32_t)USBx_BASE + USB_OTG_PCGCCTL_BASE)</br>
//note USBx_BASE is not defined, we need define it before we use these registers.</br>
```</br>
|_ macros: mask/unmask, clear interrupt flag..</br>
|_ functions: </br>
```</br>
HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);</br>
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);</br>
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_ModeTypeDef mode);</br>
HAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx, uint8_t speed);</br>
HAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num);</br>
HAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_ActivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len);</br>
void             *USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len);</br>
HAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);</br>
HAL_StatusTypeDef USB_SetDevAddress(USB_OTG_GlobalTypeDef *USBx, uint8_t address);</br>
HAL_StatusTypeDef USB_DevConnect(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_DevDisconnect(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_ActivateSetup(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx, uint8_t *psetup);</br>
uint8_t           USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx);</br>
uint32_t          USB_GetMode(USB_OTG_GlobalTypeDef *USBx);</br>
uint32_t          USB_ReadInterrupts(USB_OTG_GlobalTypeDef *USBx);</br>
uint32_t          USB_ReadDevAllOutEpInterrupt(USB_OTG_GlobalTypeDef *USBx);</br>
uint32_t          USB_ReadDevOutEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);</br>
uint32_t          USB_ReadDevAllInEpInterrupt(USB_OTG_GlobalTypeDef *USBx);</br>
uint32_t          USB_ReadDevInEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);</br>
void              USB_ClearInterrupts(USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt);</br>
</br>
HAL_StatusTypeDef USB_HostInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);</br>
HAL_StatusTypeDef USB_InitFSLSPClkSel(USB_OTG_GlobalTypeDef *USBx, uint8_t freq);</br>
HAL_StatusTypeDef USB_ResetPort(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_DriveVbus(USB_OTG_GlobalTypeDef *USBx, uint8_t state);</br>
uint32_t          USB_GetHostSpeed(USB_OTG_GlobalTypeDef *USBx);</br>
uint32_t          USB_GetCurrentFrame(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx,</br>
                              uint8_t ch_num,</br>
                              uint8_t epnum,</br>
                              uint8_t dev_address,</br>
                              uint8_t speed,</br>
                              uint8_t ep_type,</br>
                              uint16_t mps);</br>
HAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_HCTypeDef *hc);</br>
uint32_t          USB_HC_ReadInterrupt(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_HC_Halt(USB_OTG_GlobalTypeDef *USBx, uint8_t hc_num);</br>
HAL_StatusTypeDef USB_DoPing(USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num);</br>
HAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx);</br>
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx);</br>
```</br>
</br>
### stm32l4xx_hal_pcd.h/c</br>
built on top of stm32l4xx_ll_usb.h/c</br>
specialized on PCD mode.</br>
enumerations:</br>
|_ PCD_StateTypeDef: reset/ready/error/busy/timeout</br>
|_ PCD_LPM_StateTypeDef</br>
|_ PCD_LPM_MsgTypeDef</br>
|_ PCD_BCD_MsgTypeDef: (BCD: binary-coded decimal), msg is in bcd format.</br>
</br>
data structures:</br>
|_ PCD_TypeDef from stm32l476xx.h.</br>
|_ PCD_InitTypeDef from stm32l476xx.h.</br>
|_ PCD_EPTypeDef from stm32l476xx.h.</br>
|_ PCD_HandleTypeDef define status and callbacks</br>
</br>
macros:</br>
__HAL_PCD_ENABLE</br>
__HAL_PCD_DISABLE</br>
__HAL_PCD_GET_FLAG</br>
__HAL_PCD_CLEAR_FLAG</br>
__HAL_PCD_IS_INVALID_INTERRUPT</br>
__HAL_PCD_UNGATE_PHYCLOCK</br>
__HAL_PCD_GATE_PHYCLOCK</br>
__HAL_PCD_IS_PHY_SUSPENDED</br>
__HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_IT</br>
__HAL_USB_OTG_FS_WAKEUP_EXTI_DISABLE_IT</br>
__HAL_USB_OTG_FS_WAKEUP_EXTI_GET_FLAG</br>
__HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG</br>
__HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_EDGE</br>
</br>
functions:</br>
```</br>
//initialization de-init functions</br>
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);</br>
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);</br>
</br>
//IO operations: non-blocking mode, interrupt mode.</br>
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);</br>
</br>
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);//this is the core function.</br>
</br>
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);</br>
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);</br>
</br>
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);</br>
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);</br>
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);</br>
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);</br>
</br>
//PCD control functions.</br>
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);</br>
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);</br>
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);</br>
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);</br>
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);</br>
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);</br>
uint16_t          HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);</br>
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);</br>
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);</br>
```</br>
</br>
### middleware</br>
usbd_def.h</br>
- constants</br>
- data structures</br>
  |_USBD_SetupReqTypedef</br>
  |_USBD_ClassTypeDef: callbacks</br>
  |_USBD_DescriptorsTypeDef: callbacks</br>
  |_USBD_EndpointTypeDef</br>
  |_USBD_HandleTypeDef</br>
usbd_core.h/c</br>
```</br>
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id);</br>
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev);</br>
USBD_StatusTypeDef USBD_Start  (USBD_HandleTypeDef *pdev);</br>
USBD_StatusTypeDef USBD_Stop   (USBD_HandleTypeDef *pdev);</br>
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass);</br>
</br>
USBD_StatusTypeDef USBD_RunTestMode (USBD_HandleTypeDef  *pdev);</br>
USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx);</br>
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx);</br>
</br>
USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup);</br>
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev , uint8_t epnum, uint8_t *pdata);</br>
USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev , uint8_t epnum, uint8_t *pdata);</br>
</br>
USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef  *pdev);</br>
USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef  *pdev, USBD_SpeedTypeDef speed);</br>
USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef  *pdev);</br>
USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef  *pdev);</br>
</br>
USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef  *pdev);</br>
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_HandleTypeDef  *pdev, uint8_t epnum);</br>
USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(USBD_HandleTypeDef  *pdev, uint8_t epnum);</br>
</br>
USBD_StatusTypeDef USBD_LL_DevConnected(USBD_HandleTypeDef  *pdev);</br>
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef  *pdev);</br>
</br>
/* USBD Low Level Driver */</br>
USBD_StatusTypeDef  USBD_LL_Init (USBD_HandleTypeDef *pdev);</br>
USBD_StatusTypeDef  USBD_LL_DeInit (USBD_HandleTypeDef *pdev);</br>
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev);</br>
USBD_StatusTypeDef  USBD_LL_Stop (USBD_HandleTypeDef *pdev);</br>
USBD_StatusTypeDef  USBD_LL_OpenEP  (USBD_HandleTypeDef *pdev,</br>
                                      uint8_t  ep_addr,</br>
                                      uint8_t  ep_type,</br>
                                      uint16_t ep_mps);</br>
</br>
USBD_StatusTypeDef  USBD_LL_CloseEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr);</br>
USBD_StatusTypeDef  USBD_LL_FlushEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr);</br>
USBD_StatusTypeDef  USBD_LL_StallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr);</br>
USBD_StatusTypeDef  USBD_LL_ClearStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr);</br>
uint8_t             USBD_LL_IsStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr);</br>
USBD_StatusTypeDef  USBD_LL_SetUSBAddress (USBD_HandleTypeDef *pdev, uint8_t dev_addr);</br>
USBD_StatusTypeDef  USBD_LL_Transmit (USBD_HandleTypeDef *pdev,</br>
                                      uint8_t  ep_addr,</br>
                                      uint8_t  *pbuf,</br>
                                      uint16_t  size);</br>
</br>
USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,</br>
                                           uint8_t  ep_addr,</br>
                                           uint8_t  *pbuf,</br>
                                           uint16_t  size);</br>
</br>
uint32_t USBD_LL_GetRxDataSize  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr);</br>
void  USBD_LL_Delay (uint32_t Delay);</br>
```</br>
</br>
usbd_ctlrq.h/c</br>
```</br>
USBD_StatusTypeDef  USBD_StdDevReq (USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef  *req);</br>
USBD_StatusTypeDef  USBD_StdItfReq (USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef  *req);</br>
USBD_StatusTypeDef  USBD_StdEPReq  (USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef  *req);</br>
</br>
</br>
void USBD_CtlError  (USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef *req);</br>
</br>
void USBD_ParseSetupRequest (USBD_SetupReqTypedef *req, uint8_t *pdata);</br>
</br>
void USBD_GetString         (uint8_t *desc, uint8_t *unicode, uint16_t *len);</br>
```</br>
</br>
usbd_ioreq.h</br>
```</br>
USBD_StatusTypeDef  USBD_CtlSendData (USBD_HandleTypeDef *pdev,</br>
                               uint8_t *pbuf,</br>
                               uint16_t len);</br>
</br>
USBD_StatusTypeDef  USBD_CtlContinueSendData (USBD_HandleTypeDef  *pdev,</br>
                               uint8_t *pbuf,</br>
                               uint16_t len);</br>
</br>
USBD_StatusTypeDef USBD_CtlPrepareRx (USBD_HandleTypeDef  *pdev,</br>
                               uint8_t *pbuf,</br>
                               uint16_t len);</br>
</br>
USBD_StatusTypeDef  USBD_CtlContinueRx (USBD_HandleTypeDef  *pdev,</br>
                              uint8_t *pbuf,</br>
                              uint16_t len);</br>
</br>
USBD_StatusTypeDef  USBD_CtlSendStatus (USBD_HandleTypeDef  *pdev);</br>
</br>
USBD_StatusTypeDef  USBD_CtlReceiveStatus (USBD_HandleTypeDef  *pdev);</br>
uint32_t  USBD_GetRxCount (USBD_HandleTypeDef *pdev, uint8_t ep_addr);</br>
```</br>
 </br>
## how does it work</br>
driver send control information to ep0</br>
hardware interrupt will drive the driver code.</br>
</br>
- when MCU up or USB plug in, a session request interrupt is generated, and pc driver will send query and setup packages.</br>
and eastablish the control pipe (with EP0)</br>
- when usb plug off or MCU power down, session terminated and will generate an interrupt.</br>
powered state</br>
- host will send reset signalling and begin enumeration. It will generate reset interrupt and enumeration done interrupt.</br>
and enter in default state.</br>
</br>
- default state: set_address, not other command is accepted.</br>
</br>
- addressed state: ready to answer usb transaction on the address.</br>
</br>
- when usb bus 3ms idle, it will enter suspended state and generate an interrupt</br>
- resume from the host to clear the suspended state.</br>
</br>
1 in/out control ep0</br>
5 in data ep</br>
5 out data ep.</br>
</br>
events:</br>
</br>
• Transfer completed interrupt, indicating that data transfer was completed on both the</br>
application (AHB) and USB sides</br>
• Setup stage has been done (control-out only)</br>
• Associated transmit FIFO is half or completely empty (in endpoints)</br>
• NAK acknowledge has been transmitted to the host (isochronous-in only)</br>
• IN token received when Tx FIFO was empty (bulk-in/interrupt-in only)</br>
• Out token received when endpoint was not yet enabled</br>
• Babble error condition has been detected</br>
• Endpoint disable by application is effective</br>
• Endpoint NAK by application is effective (isochronous-in only)</br>
• More than 3 back-to-back setup packets were received (control-out only)</br>
• Timeout condition detected (control-in only)</br>
• Isochronous out packet has been dropped, without generating an interrup</br>
</br>
how does the 5 end point cooperate to perform a transaction</br>
divide into packets and assemble packets.</br>
</br>
</br>
</br>
</br>
</br>
</br>
	</br>
		   </br>
		   </br>
		</br>
		</br>
</br>
</br>
</br>
</br>
