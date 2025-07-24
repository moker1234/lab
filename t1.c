#include "Wiper.h"
#include "main.h"
#include <stdlib.h>

void SystemClock_Config(void);

extern TIM_PWM_CONFIG tim3_pconf; 
//extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];
extern Motor motor;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

//ADC
static float V_AD1 = 0.0; 
static uint16_t adcValue = 0;
static float SPRAY_AD2 = 0.0; 
static float SPRAY_AD3 = 0.0; 
static float SPRAY_AD4 = 0.0; 
uint8_t OCFlag = 0;

//程序运行
static uint16_t programRunCount = 0;

//epprom Data

#define  DATA_Size			16
#define  EEP_Firstpage      0x00
uint8_t I2c_Buf_Write[DATA_Size];
uint8_t I2c_Buf_Read[DATA_Size];

// roundoff struct
struct RS485_RoundOff 
{
	RS485_State state;
	uint32_t blockingTime;
	uint32_t tickstart;
	uint8_t buffLength;
	uint8_t buffCount;
	uint8_t txbuffer[16][64];
	uint8_t buffstart;
	uint8_t buffend;
}static uart1_roundoff = {ModbusReady, 20, 0, 8, 10, {{0}}, 0, 0};
// 485 request typedef enum
typedef enum
{
	RS485RequestRead = 0,
	RS485WriteOpeningRatio
}RS485_Command;
static void RS485_MessageTxPush(RS485_Command, uint16_t);

struct LIN_RoundOff 
{
	RS485_State state;
	uint32_t blockingTime;
	uint32_t tickstart;
	uint8_t buffLength;
	uint8_t buffCount;
	uint8_t txbuffer[16][64];
	uint8_t buffstart;
	uint8_t buffend;
}static usart2_roundoff = {ModbusReady, 20, 0, 8, 10, {{0}}, 0, 0};
typedef enum
{
	RequestRead = 0,
	WriteOpeningRatio
}LIN_Command;
static uint8_t ms100TimingFlag = 0;//1 进入100MS
static uint8_t btState=7;//7 复位

static uint32_t tickstart_2 = 0;
static uint32_t tickstart_3 = 0;
 uint8_t sprayFlag1 = 0; //进入喷淋
 uint8_t sprayTimeFlag = 0; //1 开始计时
 uint8_t sprayFlag = 0; //喷淋标识 1 开启喷淋
uint8_t timeDelay6s = 0; //1 延时6S判断已完成
 uint8_t wiperCount = 0;
 uint8_t wiperStartFlag = 0;

uint8_t scrubberFlag = 0;
// buttonData
struct ButtonModeData
{
	uint8_t 		ButtonSlowData;// 2.41s
	uint8_t			ButtonFastData;
	uint8_t 		ButtonGapData;
	uint8_t 		ButtonStopData;
	uint8_t			ButtonSprayData;
	uint8_t 		ButtonClerData;
	uint8_t 		ButtonAutoData;
}static buttonModeData = {0x28, 0x4B, 0x10, 0, 0x10, 0x28, 0};//待修改   

//static ButtonBaseTime bbt={2*1000, 4*1000, 8*1000, 2*1000, 2*1000, 2*1000};

static uint8_t RS485BtState = 10;//7 未选择 默认状态按钮
uint8_t gapCount =0;
static void LIN_MessageTxPush(LIN_Command, uint16_t);


static void getButtonState(void);
static void chooseButtonMode(uint8_t buttonMode);
static void SystemControl(void);
static uint8_t firstResetFlag = 0;

//过流故障
#define  Calibration_Current      5.445  //
#define  Calibration_Current1      4.5  //
uint32_t calculateDynamicThreshold(float current,float tp,float ip,float integral);
uint32_t currentThreshold = 0;
static uint8_t checkOC(void);
static uint32_t faultStartTime = 0;  // 记录故障开始时间
static uint16_t maxResult = 0;  // 记录最大结果
uint16_t GetMaxADCValue_DMA(uint16_t* data);//电流最大值
uint32_t currentThresholdSpray = 0;
static uint32_t faultStartTimeSpray = 0;  // 记录故障开始时间
//ADC数据采集
static uint16_t max_value = 0;
static uint8_t maxCount = 0;
uint16_t adc4MaxValue[6];
static uint8_t rainSignal = 0;	//雨量信号
//洗车
static void washWiper(void);
static uint8_t CenteredSensorFlag = 0;
static uint8_t CenteredSensorJudgFlag = 0; //居中传感器时间判断
static uint8_t washCount = 0;
//只有喷淋
static uint8_t onlySpray = 0;
static void sprayMode(void);
//刮刷次数+时间总计
static uint32_t timeTotal = 0;
static uint32_t minutes = 0;
static uint32_t tickstart_hour = 0;
//间隙
static uint32_t tickstart_gap = 0;
static uint32_t tickstart_wash = 0;
uint8_t washFlag = 0;
static uint8_t gapFlag = 0;
static void gapWash(void);
//添加一个上次模式变换 用来判断
static uint8_t previousState = 7;
static uint8_t lastSmartMode = 0;
struct SaveData{
	uint8_t position;
	uint32_t time;
	uint8_t num;
	uint8_t AValue;
	uint32_t timeTotal;
	uint32_t minutes;
}saveData;

struct ErrorFlag
{
	uint8_t OC;
	uint8_t RainSenor;
}errorflag = {0};
 uint32_t ADC_ConvertedValue[4];

//定义一个数据 记录30S内触发时间
int32_t timeLag = 0;//时间差
uint32_t timeArr[10] = {0};

uint32_t lastTick = 0;  // 记录上次触发的时间
uint32_t currentTick = 0;
uint8_t timeCount = 0;
uint8_t query_0[8] = {0};

uint8_t sendHead[2]={0x55,0xA3};
static uint32_t lastResetTime = 0;
static uint32_t currentResetTime = 0;
//电压采样过滤
#define FILTERSIZE (30u)
#define FILTERVALID (10u)
static int filterlist_index = 0;
static uint16_t voltage[FILTERSIZE] = {0};
static uint16_t current[FILTERSIZE] = {0};
static uint16_t current1[FILTERSIZE] = {0};
static uint16_t currentSpray[FILTERSIZE] = {0};//喷淋电机电流
static void DataUpdate(void);
static uint16_t CurrentFilter(uint16_t*);

#define ADC_TO_VOLTAGE_SLOPE 0.0854f // 斜率
#define ADC_TO_VOLTAGE_INTERCEPT -171.93f // 截距
static uint8_t firstPowerOn = 0;
//洗车持续停止0.5S
uint8_t centerSingalStopFlag =0;
static uint32_t tickstart_stop = 0;
uint8_t stopSingalFlag = 0;
//添加一个启机识别信号
uint8_t ErrorOCcount = 0;
uint32_t firstOCTime = 0;
uint8_t lastStopFlag = 0;
uint8_t currentStopFlag = 0;
static uint32_t tickstart_OC = 0;
uint8_t startFlag = 0;

static uint16_t adcValue1 = 0;
static uint8_t runingChangeFlag = 0;
static uint32_t resertTime = 0;
static uint16_t StartupOvercurrentIgnoreTime = 350;
static void isOC(void);

#define FILTER_TIME 30 // 滤波时间20ms
// 全局变量
volatile uint8_t edge_detected = 0;

uint8_t noCenterSingalFlag =0;

//7.6   复位时间判断
uint8_t edge_detectedCount = 0;
static uint32_t resetFlagLastTime = 0;
static uint32_t resetFlagCurrentTime = 0;

static void speedChange(void);
static uint32_t reset1_timestamp[10] ={0};
static uint32_t resetTotal = 0;

void Wiper_Init(void)
{
	 /* MCU Configuration--------------------------------------------------------*/
	 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	
	/* Configure the system clock */
	SystemClock_Config();
	
	/* Initialize all configured peripherals */
 MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	RS485_Device_Init();
	LIN_IDTypeConfig();
  HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,4);

	
		//待添加    读取i2c数据并修改buttonModeData
		I2C_EE_BufferRead(I2c_Buf_Read, EEP_Firstpage, DATA_Size); 
		saveData.timeTotal =(I2c_Buf_Read[1]<<8) | I2c_Buf_Read[0];//挂刷总次数
		saveData.minutes = (I2c_Buf_Read[3]<<8) | I2c_Buf_Read[2];//挂刷总时间
		//慢速档数据   6.26新加
		if(I2c_Buf_Read[4] == 0 || I2c_Buf_Read[4] > 60){
			buttonModeData.ButtonSlowData = buttonModeData.ButtonSlowData;
		}else{
			buttonModeData.ButtonFastData = I2c_Buf_Read[4];
		}
		//快速速档数据   6.26新加
		if(I2c_Buf_Read[5] == 0 || I2c_Buf_Read[5] < 20){
			buttonModeData.ButtonFastData = buttonModeData.ButtonFastData;
		}else{
			buttonModeData.ButtonFastData = I2c_Buf_Read[5];
		}
		tickstart_hour = HAL_GetTick();
		Write_DO_State(RCO2,GPIO_PIN_RESET);
		Write_DO_State(RCO3,GPIO_PIN_RESET);
		Write_DO_State(RCO4,GPIO_PIN_RESET);
		Write_DO_State(RCO5,GPIO_PIN_RESET);
		//6.19新增  PB2 常低
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void Wiper_Process(void){
	if(ms100TimingFlag == 1){
			LIN_SendMsg(huart2,0x23, query_0, 8, FrameIDType[0x31]);
			ms100TimingFlag = 0;
	}
	// update ADC
	DataUpdate();
	UART_Process();
	SystemControl();
}

/**
	* @brief System Clock Configuration
	* @retval None
	*/
void SystemClock_Config(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
	* @brief  This function is executed in case of error occurrence.
	* @retval None
	*/


#ifdef  USE_FULL_ASSERT
/**
	* @brief  Reports the name of the source file and the source line number
	*         where the assert_param error has occurred.
	* @param  file: pointer to the source file name
	* @param  line: assert_param error line source number
	* @retval None
	*/
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
	* @brief Return the state of the digital port in USER_DI_TypeDef
	*	@param USER_DI_TypeDef value
	*	@retval GPIO_PinState
	*/
GPIO_PinState Read_DI_State(USER_DI_TypeDef di)
{
	GPIO_PinState bitstatus;
	switch (di)
	{
		case RCI1:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_CPort, Ex_DI1_CPin);
		break;
		case RCI2:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_CPort, Ex_DI2_CPin);
		break;
		case RCI3:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_CPort, Ex_DI3_CPin);
		break;
		case RCI4:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_CPort, Ex_DI4_CPin);
		break;
		case RCI5:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI5_BPin);
		break;
		case CenteredSensor1:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI6_BPin);
		break;
		case FULLI:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI7_BPin);
		break;
		case LacI:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI8_BPin);
		break;
		case ButtonSlow:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI9_BPin);
		break;
		case ButtonFast:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI10_BPin);
		break;
		case ButtonGap:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI11_BPin);
		break;
		case ButtonStop:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI12_BPin);
		break;
		case ButtonSpray:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_BPort, Ex_DI13_BPin);
		break;;
		case ButtonCler:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_DPort, Ex_DI14_DPin);
		break;
		case ButtonAuto:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_CPort, Ex_DI15_CPin);
		break;
		case WMReset1:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_CPort, Ex_DI16_CPin);
		break;
		case CenteredSensor2:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_APort, Ex_DI17_APin);
		break;
		case WMReset2:
			bitstatus = HAL_GPIO_ReadPin(Ex_DI_APort, Ex_DI18_APin);
		break;
		
	}
	return bitstatus;
}

GPIO_PinState Read_DO_State(USER_DO_TypeDef _do)
{
	GPIO_PinState bitstatus;
	switch (_do)
	{
		case RCO1:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO1_APin);
			break;
		case RCO2:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO2_APin);
			break;
		case RCO3:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO3_APin);
			break;
		case RCO4:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_CPort, Ex_DO4_CPin);
			break;
		case RCO5:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_CPort, Ex_DO5_CPin);
			break;
		case WMSTop1:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO9_APin);
			break;
		case WMSTop2:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO10_APin);
			break;
		case LackLed:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_CPort, Ex_DO11_CPin);
			break;
		case FULLLed:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO12_APin);
			 break;
//		case RainSsensorPowerOn:
//			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO13_APin);//雨量传感器 //6.19删除
		case MonterSprayPowerOn:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_BPort, Ex_DO14_BPin);//喷淋电机使能
			break;
		case MCURun:
			bitstatus = HAL_GPIO_ReadPin(Ex_DO_APort, Ex_DO13_APin);//MCURUN  //6.19修改 PB2 改为PA11 
			break;
	}
	return bitstatus;
}

/**
	* @brief Write the state of the digital port in USER_DO_TypeDef
	* @param USER_DO_TypeDef value
	* @param GPIO_PinState
	* @retval None
	*/
void Write_DO_State(USER_DO_TypeDef _do, GPIO_PinState bitstatus)
{
	switch (_do)
	{
		case RCO1:
			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO1_APin, bitstatus);
		break;
		case RCO2:
			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO2_APin, bitstatus);
		break;
		case RCO3:
			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO3_APin, bitstatus);
		break;
		case RCO4:
			HAL_GPIO_WritePin(Ex_DO_CPort, Ex_DO5_CPin, bitstatus);
		break;
		case RCO5:
			HAL_GPIO_WritePin(Ex_DO_CPort, Ex_DO4_CPin, bitstatus);
		break;
		case WMSTop2:
			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO9_APin, bitstatus);
		break;
		case WMSTop1:
			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO10_APin, bitstatus);
		break;
		case LackLed:
			HAL_GPIO_WritePin(Ex_DO_CPort, Ex_DO11_CPin, bitstatus);
		break;
		case FULLLed:
			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO12_APin, bitstatus);
		break;
//		case RainSsensorPowerOn:
//			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO13_APin, bitstatus); //6.19
		break;
		case MonterSprayPowerOn:
			HAL_GPIO_WritePin(Ex_DO_BPort, Ex_DO14_BPin, bitstatus);
		break;
		case MCURun:
			HAL_GPIO_WritePin(Ex_DO_APort, Ex_DO13_APin, bitstatus);
		break;
	}
}

void RS485_MessageReadingCallback(uint8_t* data)
{
	uart1_roundoff.state = ModbusHandling;
	if (data[0] == 0x01 && data[1] == 0x05 && data[2] == 0x03)
	{

		if(data[3] == 0x01){
			Write_DO_State(MonterSprayPowerOn,GPIO_PIN_RESET);//喷淋 6.19修改 低电平有效
			return;
		}
		if(data[3] == 0x02){
			TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, data[4]);//刮雨器1
			return;
		}
		if(data[3] == 0x03){
//			TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_3, 0);//刮雨器2
			return;
		}
		if(data[3] == 0x04){//存入数据  挂刷次数 软硬件版本信息 电流保护值
			I2c_Buf_Write[0] =data[4];//挂刷次数
			I2c_Buf_Write[1] =data[5];//软件版本信息
			I2c_Buf_Write[2] =data[6];//硬件版本信息
			I2c_Buf_Write[3] =data[7];//电流保护值
			saveData.num = data[4];
			saveData.AValue = data[7];
//			I2C_EE_BufferWrite( I2c_Buf_Write, EEP_Firstpage+3, 4);
		}
		if(data[3] == 0x05){//存入数据  V 速度  t 时间 Length 周长
			I2c_Buf_Write[0] =data[4];
			I2c_Buf_Write[1] =data[5];
			I2c_Buf_Write[2] =data[6];
			
//			saveData.speed = data[4];
			saveData.position = data[5];
			saveData.time = data[6];
			
//			I2C_EE_BufferWrite( I2c_Buf_Write, EEP_Firstpage, 3);
		}
		if(data[3] == 0x06){//读取存入epprom的数据
			I2C_EE_BufferRead(I2c_Buf_Read, EEP_Firstpage, DATA_Size); 
			if (HAL_OK != RS485_Send(I2c_Buf_Read,DATA_Size))
			{
				Error_Handler();
				return;
			}
		}
		
		if(data[3] == 0xFF){
			switch(data[4])
			{
				case 0:
					RS485BtState = 0;
					break;
				case 1:
					RS485BtState = 1;
					break;
				case 2:
					RS485BtState = 2;
					break;
				case 3:
					RS485BtState = 3;
					break;
				case 4:
					RS485BtState = 4;
					break;
				case 5:
					RS485BtState = 5;
					break;
				case 6:
					RS485BtState = 6;
					break;
				case 7:
					RS485BtState = 7;
					break;
				default:
					RS485BtState = 8;
					break;
			}
		}
	}
	
//	memset(uart1_roundoff.txbuffer[uart1_roundoff.buffstart], 0, uart1_roundoff.buffLength);
//	uart1_roundoff.buffstart++;
//	uart1_roundoff.buffstart %= uart1_roundoff.buffCount;
//	uart1_roundoff.state = ModbusReady;
}

void LIN_MessageReadingCallback(uint8_t* data){
	usart2_roundoff.state = ModbusHandling;
	// ??? define the handler for result of query 
	if (data[0] == 0x00 && data[1] == 0x55 && data[2] == 0xA3 && data[3] == 0x01)
	{
			if(data[5] == 0x00){
				rainSignal = 1;
			}else if(data[5] == 0x01){
				rainSignal = 2;
			}else if(data[5] == 0x02){	
				rainSignal = 3;
			}else if(data[5] == 0x03){	
				rainSignal = 4;
			}
	}else 
	{
			rainSignal = 0;
		usart2_roundoff.state = ModbusError;
		Error_Handler();
	}
	
	memset(usart2_roundoff.txbuffer[usart2_roundoff.buffstart], 0, usart2_roundoff.buffLength);
	usart2_roundoff.buffstart++;
	usart2_roundoff.buffstart %= usart2_roundoff.buffCount;
	usart2_roundoff.state = ModbusReady;
	
}

static void RS485_MessageTxPush(RS485_Command commamd, uint16_t value)
{
	query_0[0] = (errorflag.RainSenor << 1) | errorflag.OC;
	switch (commamd)
	{
		case RS485RequestRead:
			if (HAL_OK != RS485_Send(query_0,8))
			{
    Error_Handler();
				return;
			}
		break;
		default:
			return;
	}
}

/**
	* @brief Push Prepared Message to RS485 Message Buffer
	* @param LIN_Command
	* @param any needed value for command 
	* @retval None
	*/
static void LIN_MessageTxPush(LIN_Command command, uint16_t value)
{
	switch (command)
	{
		case RequestRead:
//			LIN_SendMsg(huart2,0x23, query_0, 8, FrameIDType[0x31]); //主机发送读取雨量信号
		break;
		default:
			return;
	}
}

/**
	* @brief timer, timeout interval:1ms
	* @note implement the interface of TIM2 Timeout Callback
	*/
void TIM2_TimeoutCallback(void)
{
	if(programRunCount % 200 == 0){
						ms100TimingFlag = 1;

	}
		// 每次循环都更新引脚状态
	if (programRunCount <= 500) {
			Write_DO_State(MCURun, GPIO_PIN_SET);  // 前500次高电平
	} else {
			Write_DO_State(MCURun, GPIO_PIN_RESET); // 后500次低电平
	}
	// 计数器管理
	programRunCount = (programRunCount + 1) % 1000;  // 自动归零
	//
	if(GPIO_PIN_RESET == Read_DI_State(WMReset1)){
		resetFlagCurrentTime = HAL_GetTick();
//		if(startFlag == 0){
//			edge_detected = 1;
//			return;
//		}
		if(resetFlagCurrentTime - resetFlagLastTime >FILTER_TIME){
			edge_detected = 1;
			edge_detectedCount++;
			reset1_timestamp[resetTotal%10] = HAL_GetTick();  // 记录到达复位时间戳
			resetTotal++;
			if(wiperStartFlag == 1){
				wiperCount++;
			}
		}
		resetFlagLastTime = resetFlagCurrentTime;
	}else if(GPIO_PIN_SET == Read_DI_State(WMReset1)){
		edge_detected = 0;
	}
}

//系统控制
void SystemControl(void){
	//速度调节
	if(edge_detected == 1){
		speedChange();
	}
	if(GPIO_PIN_SET == Read_DI_State(LacI) && GPIO_PIN_RESET == Read_DI_State(FULLI)){//有水满+无缺水信号  水满灯亮
		Write_DO_State(FULLLed,GPIO_PIN_SET);
		Write_DO_State(LackLed,GPIO_PIN_RESET);
	}else if(GPIO_PIN_RESET == Read_DI_State(LacI) && GPIO_PIN_SET == Read_DI_State(FULLI)){//有缺水信号+无水满信号 缺水灯亮   
		Write_DO_State(FULLLed,GPIO_PIN_RESET);
		Write_DO_State(LackLed,GPIO_PIN_SET);
	}else{
		Write_DO_State(FULLLed,GPIO_PIN_RESET);
		Write_DO_State(LackLed,GPIO_PIN_RESET);
	} 
	//4.30新增
		if(errorflag.OC == 1 && firstResetFlag == 3){
		//前一次为 停止档   当前为慢速档 允许重启
		getButtonState();
		if(previousState == ButtonStopMode){
			Write_DO_State(WMSTop1,GPIO_PIN_RESET);
			CenteredSensorJudgFlag = 0;
			washFlag = 0;
			gapFlag = 0;
			firstResetFlag = 0;
			centerSingalStopFlag = 0;
			stopSingalFlag = 1;
			runingChangeFlag = 1; 
			OCFlag =0;
			startFlag =0;
			CenteredSensorFlag = 0;
			resetTotal = 0;
				noCenterSingalFlag = 0;
			} 
	}
	onlySpray = 1; 
//			//喷淋模式单独拿出
	if(GPIO_PIN_RESET == Read_DI_State(ButtonSpray) && sprayTimeFlag == 0){
		sprayFlag1 =1;
		sprayFlag = 1;
		sprayTimeFlag = 1;
	}
	switch(firstResetFlag)
	{	
		case 0:
			//Wiper Reset   首先判断复位信号  上电先进行复位
			if(0 == edge_detected){
				Write_DO_State(WMSTop1,GPIO_PIN_RESET);
				TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonSlowData);
				if(startFlag == 0){//初次上电
					lastStopFlag = 0;
					currentStopFlag = 1;
					startFlag =1 ;
					resertTime = HAL_GetTick();//重启计时
				}
				if(HAL_GetTick() - resertTime <= StartupOvercurrentIgnoreTime){
					runingChangeFlag = 1;
				}else if(HAL_GetTick() - resertTime > StartupOvercurrentIgnoreTime){
					runingChangeFlag = 0;
				}
			}else if(1 == edge_detected){
						// 超过最小时间间隔，认为是有效的复位信号
					TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
					Write_DO_State(WMSTop1,GPIO_PIN_SET);
					firstResetFlag = 1;
						// 更新上次复位的时间戳
					lastResetTime = currentResetTime;
					runingChangeFlag = 0;
					resertTime = HAL_GetTick();//重启计时
			}
			break;
		case 1:
			getButtonState();
			//   当洗车和喷淋同时选择时 洗车优先
			if(GPIO_PIN_RESET == Read_DI_State(ButtonSpray) && GPIO_PIN_RESET == Read_DI_State(ButtonCler)){
				sprayFlag1 =0;
				sprayFlag = 0;
				sprayTimeFlag = 0;
			}
			lastStopFlag = currentStopFlag;
			if(btState !=10 && btState != ButtonStopMode){
				currentStopFlag = 1;
			}else{
				currentStopFlag = 0;
			}
			//模式转换后先复位   模式转换 先复位在变速
			if(previousState != btState){
				CenteredSensorJudgFlag = 0;
				washFlag = 0;
				gapFlag = 0;
				firstResetFlag = 0;
				centerSingalStopFlag = 0;
				stopSingalFlag = 1;
				startFlag =0;
				CenteredSensorFlag = 0;
				noCenterSingalFlag = 0;
				resetTotal = 0;
				return;
			}
			chooseButtonMode(btState);
			break;
	}
		//非洗车进入喷淋
	if(btState != ButtonClerMode){
//		sprayMode();
	}
	isOC();
}

	//get Button State
static void getButtonState(){
	previousState = btState;
	if(GPIO_PIN_RESET == Read_DI_State(ButtonStop) || RS485BtState == 3){
		btState = ButtonStopMode;
		return;
	}
	else if(GPIO_PIN_RESET == Read_DI_State(ButtonSlow) || RS485BtState == 0){
		btState = ButtonSlowMode;
		return;
	}
	else if(GPIO_PIN_RESET == Read_DI_State(ButtonFast)|| RS485BtState == 1){
		btState = ButtonFastMode;
		return;
	}
	else if(GPIO_PIN_RESET == Read_DI_State(ButtonGap) || RS485BtState == 2){
		btState = ButtonGapMode;
		return;
	}
	else if(RS485BtState == 4){//GPIO_PIN_RESET == Read_DI_State(ButtonSpray) || 
		btState = ButtonSprayMode;
		return;
	}
	else if(GPIO_PIN_RESET == Read_DI_State(ButtonCler) || RS485BtState == 5){
		btState = ButtonClerMode;
		return;
	}
	else if(GPIO_PIN_RESET == Read_DI_State(ButtonAuto) || RS485BtState == 6){
		if(rainSignal == 0){
			btState = ButtonAutoMode;
		}else if(rainSignal == 1){
						btState = ButtonAutoMode;
		}else if(rainSignal == 2){
						btState = ButtonGapMode;
		}else if(rainSignal == 3){
						btState = ButtonSlowMode;
		}else if(rainSignal == 4){
						btState = ButtonFastMode;
		}
		return;
	}
	else{//未检测到所有档位信号
		btState = 10; 
	}
}

static void chooseButtonMode(uint8_t buttonMode){
	onlySpray = 2;
	//根据按钮状态选择模式
	switch(buttonMode){
		case ButtonSlowMode://慢速
			if(HAL_GetTick() - resertTime <= StartupOvercurrentIgnoreTime){
				runingChangeFlag = 1;
			}else if(HAL_GetTick() - resertTime > StartupOvercurrentIgnoreTime){
				
				runingChangeFlag = 0;
			}
			Write_DO_State(WMSTop1,GPIO_PIN_RESET);
			TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonSlowData);
		break;
		case ButtonFastMode://快速
			if(HAL_GetTick() - resertTime <= StartupOvercurrentIgnoreTime){
				runingChangeFlag = 1;
			}else if(HAL_GetTick() - resertTime > StartupOvercurrentIgnoreTime){
				
				runingChangeFlag = 0;
			}
			Write_DO_State(WMSTop1,GPIO_PIN_RESET);
			TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonFastData);
		break;
		case ButtonGapMode://间隙   6S 
//			gapWash();
		break;
		case ButtonStopMode://停止	
			//停止档位选择喷淋
			if(sprayFlag1 == 1){
				onlySpray = 1; 
			}else{
				if(HAL_GetTick() - resertTime < 1000){
					Write_DO_State(WMSTop1,GPIO_PIN_SET);
					TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
				}else{
					Write_DO_State(WMSTop1,GPIO_PIN_RESET);
					TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
				}
			}
			if(stopSingalFlag == 1){
				I2c_Buf_Write[0] = saveData.timeTotal & 0xFF;//次数
				I2c_Buf_Write[1] = saveData.timeTotal >> 8;//次数
				minutes = (tickstart_hour - HAL_GetTick()) / 1000 / 60;
				saveData.minutes = saveData.minutes + minutes;
				I2c_Buf_Write[2] = saveData.minutes & 0xFF;//时间
				I2c_Buf_Write[3] = saveData.minutes >> 8;//时间
				tickstart_hour = HAL_GetTick();
				I2c_Buf_Write[4] = buttonModeData.ButtonSlowData;//慢速
				I2c_Buf_Write[5] = buttonModeData.ButtonFastData;//快速
				I2C_EE_BufferWrite(I2c_Buf_Write, EEP_Firstpage, DATA_Size); 
				stopSingalFlag = 0;
			}
		break;
		case ButtonClerMode://洗车
				washWiper();			
		break;
		case ButtonAutoMode://智能
				TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
				onlySpray = 1; 
		break;
		case 10:
				onlySpray = 1; 
		break;
	}

}
//间隙
static void gapWash(void){
	if(1 == edge_detected && gapFlag == 0){
		gapFlag = 1;
	}
	if(gapFlag == 1){
		Write_DO_State(WMSTop1,GPIO_PIN_RESET);
		TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonSlowData);
		tickstart_gap = HAL_GetTick();
		gapFlag = 2;
		gapCount = 0;
		lastStopFlag = 0;
		currentStopFlag = 1;
		runingChangeFlag = 1;
		resertTime = HAL_GetTick();//重启计时
	}
	if(HAL_GetTick() - resertTime <= StartupOvercurrentIgnoreTime){
				runingChangeFlag = 1;
			}else if(HAL_GetTick() - resertTime > StartupOvercurrentIgnoreTime){
				
				runingChangeFlag = 0;
			}
	
	if(gapFlag ==2){
			if(HAL_GetTick() - tickstart_gap> 600 && 1 == edge_detected){
				Write_DO_State(WMSTop1,GPIO_PIN_SET);
				TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
				gapFlag = 3;
				tickstart_stop = HAL_GetTick();
			}
		}
		if(gapFlag == 3 && HAL_GetTick() -tickstart_stop <1000){
				Write_DO_State(WMSTop1,GPIO_PIN_SET);
				TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
				lastStopFlag = 0;
				currentStopFlag = 0;
		}
	if( HAL_GetTick() - tickstart_gap >6*1000){
//		gapFlag = 1;
	}
}


//洗车
static void washWiper(void){
	switch(CenteredSensorJudgFlag){
		case 0:
			if(washFlag == 0){
				tickstart_wash = HAL_GetTick();
				resertTime = HAL_GetTick();//重启计时
				washFlag =1;
				centerSingalStopFlag = 0;
			}
			Write_DO_State(WMSTop1,GPIO_PIN_RESET);
			TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonSlowData);
			if(HAL_GetTick() - resertTime <= StartupOvercurrentIgnoreTime){
				runingChangeFlag = 1;
			}else if(HAL_GetTick() - resertTime > StartupOvercurrentIgnoreTime){
				runingChangeFlag = 0;
			}
			lastStopFlag = 0;
			currentStopFlag = 1;
			//喷淋电机使能
			Write_DO_State(MonterSprayPowerOn,GPIO_PIN_RESET);//6.19修改低电平有效

			//居中传感器信号检查
			if(GPIO_PIN_RESET == Read_DI_State(CenteredSensor1)){
				CenteredSensorFlag = 1;
			}
			//运行一圈
			if(1 == edge_detected){		
				if(HAL_GetTick() - tickstart_wash >600){
					lastStopFlag = 0;
					currentStopFlag = 0;
					CenteredSensorJudgFlag = 1;
					TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonSlowData/2);//速度降半
					washCount = 0;
					noCenterSingalFlag = 0;
					edge_detectedCount = 0;
					tickstart_wash = HAL_GetTick();
				}
			}
			break;
		case 1:
			switch(CenteredSensorFlag){
				case 0://无居中传感器
					//先挂刷一圈 第二圈的1.2S停止
					if(1 == edge_detected && noCenterSingalFlag == 0){
						if(edge_detectedCount >= 2){
							tickstart_wash = HAL_GetTick();
							noCenterSingalFlag = 1;
						}
					}
					if(HAL_GetTick() - tickstart_wash >= ((reset1_timestamp[2] - reset1_timestamp[1])*3/4) && centerSingalStopFlag ==0 && noCenterSingalFlag == 1){
								TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
								Write_DO_State(WMSTop1,GPIO_PIN_SET);
								Write_DO_State(MonterSprayPowerOn,GPIO_PIN_SET);//6.26
								tickstart_wash = HAL_GetTick();
								centerSingalStopFlag = 1;
								noCenterSingalFlag = 2;
							}
							if(centerSingalStopFlag == 1 ){
								if(HAL_GetTick() - tickstart_wash <1000){
									TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
									Write_DO_State(WMSTop1,GPIO_PIN_SET);
									Write_DO_State(MonterSprayPowerOn,GPIO_PIN_SET);//6.19修改低电平有效
									washCount = 0;
								}else{
									Write_DO_State(WMSTop1,GPIO_PIN_RESET);
								}
							}
				break;
				case 1://有居中传感器
					//居中传感器信号
					if(GPIO_PIN_RESET == Read_DI_State(CenteredSensor1) && centerSingalStopFlag ==0){
						
						TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
						Write_DO_State(WMSTop1,GPIO_PIN_SET);
						Write_DO_State(MonterSprayPowerOn,GPIO_PIN_SET);//6.19修改低电平有效
						tickstart_wash = HAL_GetTick();
						centerSingalStopFlag = 1;
					}
					if(centerSingalStopFlag == 1 ){
						if(HAL_GetTick() - tickstart_wash <1000){
								TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
								Write_DO_State(WMSTop1,GPIO_PIN_SET);
								Write_DO_State(MonterSprayPowerOn,GPIO_PIN_SET);//6.19修改低电平有效
						}else{
							Write_DO_State(WMSTop1,GPIO_PIN_RESET);
						}
					}
				break;
			}
			break;
	}
}

static void sprayMode(){//按键触发一次就重新计数
	//喷淋模式
	if(sprayFlag == 1){
		//6S一次循环
		if(sprayTimeFlag == 1){
			tickstart_2 = HAL_GetTick();// 记录当前时间
			sprayTimeFlag = 2;// 标记已开始计时
			Write_DO_State(MonterSprayPowerOn,GPIO_PIN_RESET);// 开启喷淋//6.19修改低电平有效
		}
		if(HAL_GetTick() - tickstart_2 > 6*1000){
			Write_DO_State(MonterSprayPowerOn,GPIO_PIN_SET);//6.19修改低电平有效
			wiperStartFlag = 1;
			wiperCount = 0;
			sprayTimeFlag = 3;
			sprayFlag = 2;
		}
		if(onlySpray == 1){
			Write_DO_State(WMSTop1,GPIO_PIN_RESET);
			TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonSlowData);
			lastStopFlag = 0;
			currentStopFlag = 1;
			runingChangeFlag = 1;
			resertTime = HAL_GetTick();//重启计时
		}
	}
	if(sprayFlag == 2){
		if(onlySpray ==1){
			if(HAL_GetTick() - resertTime <= StartupOvercurrentIgnoreTime){
				runingChangeFlag = 1;
			}else if(HAL_GetTick() - resertTime > StartupOvercurrentIgnoreTime){
				
				runingChangeFlag = 0;
			}
			if(wiperCount == 3){
				wiperStartFlag = 0;
				sprayFlag1 = 0;
				sprayFlag = 0;
				TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
				Write_DO_State(WMSTop1,GPIO_PIN_SET);
				tickstart_2 = HAL_GetTick();
				lastStopFlag = 0;
				currentStopFlag = 0;
			}
			return;
		}
		wiperStartFlag = 0;
		sprayFlag1 = 0;
		sprayFlag = 0;
		tickstart_2 = HAL_GetTick();
		lastStopFlag = 0;
		currentStopFlag = 0;
		return;
	}
	
	//喷淋模式下6S后进行判断
	if(GPIO_PIN_SET == Read_DI_State(ButtonSpray) && sprayTimeFlag == 3){
		if(HAL_GetTick() - tickstart_2 > 2*1000){
			sprayTimeFlag = 0;
		}
	}
}

static void isOC(void){
	
	//过流检测   出现故障 直接停机
	if(lastStopFlag == 0 && currentStopFlag == 1){
		//计时
		tickstart_OC = HAL_GetTick();
	}
	if(HAL_GetTick() -  tickstart_OC> 100 && runingChangeFlag == 0){
		OCFlag = checkOC();
	}
	
	if(OCFlag == 1){
		errorflag.OC = 1;
		TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
		Write_DO_State(WMSTop1,GPIO_PIN_SET);
		Write_DO_State(MonterSprayPowerOn,GPIO_PIN_SET);
		
		 // 记录第一次 OC 故障的时间
		if (ErrorOCcount == 0) {
				firstOCTime = HAL_GetTick();
		}
			ErrorOCcount++;
			tickstart_OC = HAL_GetTick();
			if (ErrorOCcount >= 3 && (HAL_GetTick() - firstOCTime) <= 3*1000) {
					firstResetFlag = 3;
			}else if(ErrorOCcount< 3){
					firstResetFlag = 3;
			}
			return;
	}
	//检查故障次数 及时间 是否超过5分钟 进行重置
	if(ErrorOCcount < 4 && HAL_GetTick() - firstOCTime > 5*1000){
		ErrorOCcount = 0;
		firstOCTime = 0;
	}
}


uint16_t GetMaxADCValue_DMA(uint16_t* data){
	
//ADC数据采集
static uint16_t max_value = 0;
		max_value = 0;
	 for (uint16_t i = 1; i < 30; i++) {
        if (current[i] > max_value) {
            max_value = data[i]; // 更新最大值
        }
    }
    return max_value;
}

//获得电流限值  6.19 修改
uint32_t calculateDynamicThreshold(float current,float tp,float ip,float integral){
	 if (current <= 0) {
        return 300; // 默认返回300ms防止除零或无效值
    }
    
    float ratio = current / ip;
    float denominator = powf(ratio, integral) - 1.0f;
    
    if (denominator <= 0) {
        return 300; // 防止计算结果为负或无穷大
    }
    
    float thresholdSeconds = tp / denominator; 
    // 转换为毫秒并确保最小阈值（例如不低于10ms）
    uint32_t thresholdMs = (uint32_t)(thresholdSeconds * 1000);
    return (thresholdMs < 10) ? 10 : thresholdMs; // 设置最小阈值限制
}


//过流故障检测
static uint8_t checkOC(void) {
    // 读取 ADC 采样值并转换为实际电压或电流
    adcValue = CurrentFilter(voltage);
		V_AD1 = ADC_TO_VOLTAGE_SLOPE * adcValue + ADC_TO_VOLTAGE_INTERCEPT;
	adcValue1 =  GetMaxADCValue_DMA(currentSpray);
		SPRAY_AD2 =(float)  GetMaxADCValue_DMA(currentSpray)/4096*(float)Calibration_Current1;//喷淋电机
		SPRAY_AD3 =(float) GetMaxADCValue_DMA(current)/4096*(float)Calibration_Current;
//		SPRAY_AD4 =(float) GetMaxADCValue_DMA(current1)/4096*(float)Calibration_Current; //忽略第二路
    saveData.AValue = 3;
		//喷淋电机
	   if (SPRAY_AD2 >2) {
        // 如果是第一次超过阈值，记录故障开始时间
        if (faultStartTimeSpray == 0) {
           faultStartTimeSpray = HAL_GetTick();
						currentThresholdSpray  = calculateDynamicThreshold(SPRAY_AD2,1,1.9,0.02); //
        }

        // 如果持续时间超过 0.3 秒，返回故障状态
        if (HAL_GetTick() - faultStartTimeSpray  >= currentThresholdSpray ) {  // 300ms = 0.3秒
					  faultStartTimeSpray  = 0;
            return 1;  // 故障
        }
    } else {
        // 如果未超过阈值，重置故障开始时间
        faultStartTimeSpray  = 0;
				currentThresholdSpray  = 300; // 重置为默认值
    }
    //雨刷电机 检查是否超过阈值
    if (SPRAY_AD3 > saveData.AValue || SPRAY_AD4 > saveData.AValue) {
        // 如果是第一次超过阈值，记录故障开始时间
        if (faultStartTime == 0) {
           faultStartTime = HAL_GetTick();
						currentThreshold = calculateDynamicThreshold(SPRAY_AD3,1.8,2.4,0.03);
        }

        // 如果持续时间超过 0.3 秒，返回故障状态
        if (HAL_GetTick() - faultStartTime >= currentThreshold) {  // 300ms = 0.3秒
					  faultStartTime = 0;
            return 1;  // 故障
        }
    } else {
        // 如果未超过阈值，重置故障开始时间
        faultStartTime = 0;
				currentThreshold = 300; // 重置为默认值
    }

    return 0;  // 正常
}

static void DataUpdate(void){
	voltage[filterlist_index] = ADC_ConvertedValue[0];
	currentSpray[filterlist_index] = ADC_ConvertedValue[1];
		current[filterlist_index] = ADC_ConvertedValue[2];
		current1[filterlist_index] = ADC_ConvertedValue[3];
	filterlist_index++;
	filterlist_index %= FILTERSIZE;
}

static uint16_t CurrentFilter(uint16_t*filterlist){
		uint32_t Sum = 0;
	int start = (FILTERSIZE - FILTERVALID) / 2;
	int end = start + FILTERVALID;
	uint16_t temp;
	uint16_t templist[FILTERSIZE] = {0};
	memcpy(templist, filterlist, 2 * FILTERSIZE);
	
	for (int i = 1; i < FILTERSIZE; i++) {
		for (int j = i; j > 0; j--) {
			if (templist[j] > templist[j-1]) {
					temp = templist[j];
					templist[j] = templist[j-1];
					templist[j-1] = temp;
				}
			}
		}
	
		for (int i = start; i < end; i++) {
			Sum += templist[i];
		}

		return (Sum / FILTERVALID);
}


static void speedChange(void){
		currentTick = HAL_GetTick();  // 获取当前时间
		timeLag = currentTick - lastTick;
		if(currentTick - lastTick < 800) {
				 return;
		}
		lastTick = currentTick;  // 更新上次触发的时间
		// 例如，切换LED状态或执行其他操作
		saveData.timeTotal++;//挂刷次数计数
		timeTotal++;
		if(btState == ButtonSlowMode){
			if(timeLag > 2500){
				if(timeLag > 2700){
					buttonModeData.ButtonSlowData = buttonModeData.ButtonSlowData + 4;
				}else{
					buttonModeData.ButtonSlowData = buttonModeData.ButtonSlowData + 1;
				}
				if(buttonModeData.ButtonSlowData >= 100){
					buttonModeData.ButtonSlowData = 100;
				}
			}else if(timeLag < 2370){
				if(timeLag < 2000){
					buttonModeData.ButtonSlowData = buttonModeData.ButtonSlowData - 4;
				}else{
					buttonModeData.ButtonSlowData = buttonModeData.ButtonSlowData - 1;
				}
				if(buttonModeData.ButtonSlowData <= 5){
					buttonModeData.ButtonSlowData = 5;
				}
			}
		}else if(btState == ButtonFastMode){
			if(timeLag > 1540){
				if(timeLag > 1600){
					buttonModeData.ButtonFastData = buttonModeData.ButtonFastData + 4;
				}else{
					buttonModeData.ButtonFastData = buttonModeData.ButtonFastData + 1;
				}
				if(buttonModeData.ButtonFastData >= 100){
					buttonModeData.ButtonFastData = 100;
				}
			}else if(timeLag < 1480){
				if(timeLag < 1400){
					buttonModeData.ButtonFastData = buttonModeData.ButtonFastData - 4;
				}else{
					buttonModeData.ButtonFastData = buttonModeData.ButtonFastData - 1;
				}
				if(buttonModeData.ButtonFastData <= 5){
					buttonModeData.ButtonFastData = 5;
				}
			}
		}else if(btState == ButtonGapMode){
			gapCount++;
		}else if(btState == 7 && sprayFlag == 0){
			TIM_SetDutyRatio(&tim3_pconf, TIM_CHANNEL_2, buttonModeData.ButtonStopData);
		}
		timeArr[timeTotal%10] = timeLag;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_11) {        // 处理PC11下降沿事件
		
    }
}
