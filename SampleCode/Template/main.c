/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;

typedef enum
{
	ADC_DataState_AVERAGE = 0 ,
	ADC_DataState_MMA , 
	
	ADC_DataState_DEFAULT 	
}ADC_DataState_TypeDef;

#define ADC_CH   							(uint8_t) (2)

#define ADC_SAMPLETIME_MS					(uint16_t) (20)

#define ADC_RESOLUTION						((uint16_t)(4096u))
#define ADC_REF_VOLTAGE						((uint16_t)(3300u))	//(float)(3.3f)

#define ADC_MAX_TARGET						((uint16_t)(4095u))	//(float)(2.612f)
#define ADC_MIN_TARGET						((uint16_t)(0u))	//(float)(0.423f)

#define DUTY_MAX							(uint16_t)(100)
#define DUTY_MIN							(uint16_t)(0)
#define ADC_CONVERT_TARGET					(float)(ADC_MIN_TARGET*ADC_RESOLUTION/ADC_REF_VOLTAGE) //81.92000 
#define ADC_SUB_TARGET						(float)((ADC_MAX_TARGET-ADC_MIN_TARGET)/(DUTY_MAX-DUTY_MIN)*(ADC_RESOLUTION/ADC_REF_VOLTAGE))//5.60505 
//#define ADCInputV_Sub						(float)	((ADC_MAX_BRIGHT-ADC_MIN_BRIGHT)/(DUTY_MAX-DUTY_MIN)) //0.02737 

#define ADC_SAMPLE_COUNT 					(uint16_t)(16)		// 8
#define ADC_SAMPLE_POWER 					(uint8_t)(4)			//(5)	 	// 3	,// 2 ^ ?

#define ADC_DMA_SAMPLE_COUNT 			(uint16_t)(1)

#define ABS(X)  								((X) > 0 ? (X) : -(X)) 

#define ADC_DIGITAL_SCALE(void) 				(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 
#define ADC_CALC_DATA_TO_VOLTAGE(DATA,VREF) ((DATA) * (VREF) / ADC_DIGITAL_SCALE())


#define ADCextendSampling 					(0)
#define PDMAchannel 						(1)

uint16_t aADCxConvertedData = 0;

//uint16_t adcRawData_Target = 0;
uint16_t movingAverage_Target = 0;
uint32_t movingAverageSum_Target = 0;
uint8_t ADCDataReady = 0;

volatile uint32_t g_u32AdcIntFlag = 0;
uint16_t AVdd = 0;

ADC_DataState_TypeDef 	ADCDataState = ADC_DataState_DEFAULT;

__STATIC_INLINE uint32_t FMC_ReadBandGap(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = 0x70u;                         /* Must keep 0x70 when read Band-Gap */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */

    return FMC->ISPDAT & 0xFFF;
}

void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
}

void ADC_ReadAVdd(void)
{
    int32_t  i32ConversionData;
    int32_t  i32BuiltInData;

    ADC_POWER_ON(ADC);
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT29);
    ADC_SetExtendSampleTime(ADC, 0, 71);
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);
    g_u32AdcIntFlag = 0;
    ADC_START_CONV(ADC);

    while(g_u32AdcIntFlag == 0);
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);
		
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 29);
    SYS_UnlockReg();
    FMC_Open();
    i32BuiltInData = FMC_ReadBandGap();	

//	printf("%s : %d,%d\r\n",__FUNCTION__, i32ConversionData,i32BuiltInData);

	AVdd = 3072*i32BuiltInData/i32ConversionData;

    NVIC_DisableIRQ(ADC_IRQn);
	
}

void ReloadPDMA(void)
{
    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA_SetTransferCnt(PDMA, PDMAchannel, PDMA_WIDTH_16, ADC_DMA_SAMPLE_COUNT);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, PDMAchannel, PDMA_ADC_RX, FALSE, (uint32_t) NULL);
}

uint8_t Is_ADC_DataReady(void)
{
	return ADCDataReady;
}

void ADC_DataReady(uint8_t on)
{
	ADCDataReady = on;
}

uint16_t ADC_ModifiedMovingAverage(uint16_t data)
{
	static uint16_t cnt = 0;

	if (Is_ADC_DataReady())
	{
		ADC_DataReady(0);
		
//		printf("data : %d\r\n" , data);
		
		switch(ADCDataState)
		{
			case ADC_DataState_AVERAGE:
				movingAverageSum_Target += data;
				if (cnt++ >= (ADC_SAMPLE_COUNT-1))
				{
					cnt = 0;
					movingAverage_Target = movingAverageSum_Target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;;
					ADCDataState = ADC_DataState_MMA;
				}			
				break;
				
			case ADC_DataState_MMA:
				movingAverageSum_Target -=  movingAverage_Target;
				movingAverageSum_Target +=  data;
				movingAverage_Target = movingAverageSum_Target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;
	
//				printf("Average : %d\r\n" , movingAverage);
				break;				
		}
	}	

	return movingAverage_Target;
}


void ADC_MMA_Initial(void)
{
	ADCDataState = ADC_DataState_AVERAGE;
	movingAverageSum_Target = 0;
	movingAverage_Target = 0;
	ADC_DataReady(0);
}

uint16_t ADC_ConvertChannel(uint8_t ch)
{
	__IO uint16_t adc_value = 0;
	__IO uint16_t duty_value = 0;
	__IO uint16_t adcRawData_Target = 0;

    /* reload PDMA configuration for next transmission */
    ReloadPDMA();

//	ADC_DataReady(1);	

	adc_value = ADC_ModifiedMovingAverage(((aADCxConvertedData >>1)<<1));
	
	printf("%s : 0x%4X (%d mv)\r\n",__FUNCTION__,adc_value , ADC_CALC_DATA_TO_VOLTAGE(adc_value,AVdd));
//	printf("%s : %d\r\n",__FUNCTION__, ADC_ReadAVdd());

	
	if (adc_value <= ADC_CONVERT_TARGET)
	{
		adc_value = ADC_CONVERT_TARGET;
	}

	if (adc_value >= ADC_RESOLUTION)
	{
		adc_value = ADC_RESOLUTION;
	}

    /* Stop ADC conversion */
//    ADC_STOP_CONV(ADC);

    /* Disable PDMA function of ADC */
//    ADC_DISABLE_PDMA(ADC);

	return adc_value;
}

void ADC_InitChannel(uint8_t ch)
{
	ADC_ReadAVdd();

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /*Wait for ADC internal power ready*/
    CLK_SysTickDelay(10000);

    /* Set input mode as single-end, and Single mode*/
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS,(uint32_t) BIT0|BIT1);

    /* To sample band-gap precisely, the ADC capacitor must be charged at least 3 us for charging the ADC capacitor ( Cin )*/
    /* Sampling time = extended sampling time + 1 */
    /* 1/24000000 * (Sampling time) = 3 us */
	/*
	    printf("+----------------------------------------------------------------------+\n");
	    printf("|   ADC clock source -> PCLK1  = 48 MHz                                |\n");
	    printf("|   ADC clock divider          = 2                                     |\n");
	    printf("|   ADC clock                  = 48 MHz / 2 = 24 MHz                   |\n");
	    printf("|   ADC extended sampling time = 71                                    |\n");
	    printf("|   ADC conversion time = 17 + ADC extended sampling time = 88         |\n");
	    printf("|   ADC conversion rate = 24 MHz / 88 = 272.7 ksps                     |\n");
	    printf("+----------------------------------------------------------------------+\n");
	*/

    /* Set extend sampling time based on external resistor value.*/
    ADC_SetExtendSampleTime(ADC,(uint32_t) NULL, ADCextendSampling);

    /* reload PDMA configuration for next transmission */
    ReloadPDMA();

    /* Select ADC input channel */
    ADC_SET_INPUT_CHANNEL(ADC, 0x1 << ch);

    /* ADC enable PDMA transfer */
    ADC_ENABLE_PDMA(ADC);

    /* Start ADC conversion */
    ADC_START_CONV(ADC);

	ADC_MMA_Initial();
	
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF1_Msk)
        {
			printf("%s abort\r\n" , __FUNCTION__);
        }
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)
        {
//			printf("%s done\r\n" , __FUNCTION__);
			ADC_DataReady(1);
        }
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown PDMA interrupt !!\n");
}

void PDMA_Init(void)
{
    /* Configure PDMA peripheral mode form ADC to memory */
    /* Open PDMA Channel 1 based on PDMAchannel setting*/
    PDMA_Open(PDMA, 1 << PDMAchannel);

    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA_SetTransferCnt(PDMA, PDMAchannel, PDMA_WIDTH_16, ADC_DMA_SAMPLE_COUNT);

    /* Set source address as ADC data register (no increment) and destination address as g_i32ConversionData array (increment) */
    PDMA_SetTransferAddr(PDMA, PDMAchannel, (uint32_t)&ADC->ADPDMA, PDMA_SAR_FIX, (uint32_t)&aADCxConvertedData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, PDMAchannel, PDMA_ADC_RX, FALSE, 0);

    /* Set PDMA as single request type for ADC */
    PDMA_SetBurstType(PDMA, PDMAchannel, PDMA_REQ_SINGLE, 0);

    PDMA_EnableInt(PDMA, PDMAchannel, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);


}

void TMR3_IRQHandler(void)
{
	static uint32_t LOG = 0;
	static uint16_t CNT = 0;
	static uint16_t CNT_ADC = 0;
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);
	
		if (CNT++ >= 1000)
		{		
			CNT = 0;
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
		}

		if (CNT_ADC++ >= ADC_SAMPLETIME_MS)
		{		
			CNT_ADC = 0;
			ADC_ConvertChannel(ADC0_CH0);
		}
		
    }
}


void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}


void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
//	UART_SetTimeoutCnt(UART0, 20);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	
    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
	
    CLK_EnableModuleClock(ADC_MODULE);	
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(2));
    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);


    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    SYS->GPB_MFPL = (SYS->GPB_MFPL &~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk )) \
                    | (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1) ;

    /* Set PB.0 ~ PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT1, GPIO_MODE_INPUT);

    /* Disable the PB0 ~ PB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1);

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

    UART0_Init();

	TIMER3_Init();

    PDMA_Init();

	ADC_InitChannel(ADC0_CH0);
	
    /* Got no where to go, just loop forever */
    while(1)
    {

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
