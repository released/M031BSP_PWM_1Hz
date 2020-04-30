/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

typedef enum{
	flag_reverse = 0 ,
	
	flag_DEFAULT	
}Flag_Index;


uint8_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint8_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))



/*
	Target : 200K Freq
	DUTY : 50%
	
	SYS_CLK : 48M
	PSC : 1

	48 000 000/200 000 = PSC * (CNR + 1)
	CNR = (SYS_CLK/FREQ)/PSC - 1 = 239

	50 /100 = CMR / (CNR + 1)
	CMR = 50 * (CNR + 1)/100
	
*/

#define SYS_CLK 									(48000000ul)
#define PWM_PSC 								(1000)	
#define PWM_FREQ 								(1)	
#define PWM_DUTY                               (0)

//16 bit
#define PWM_CNR 								((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)
#define PWM_CMR 								(PWM_DUTY * (PWM_CNR + 1)/1000)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	(u32DutyCycle * (PWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(PWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))

void PWM_Set_Duty(uint16_t duty)		// 1 ~ 1000 , 0.1 % to 100%
{
    uint32_t u32NewCMR = 0;
	u32NewCMR = CalNewDutyCMR(PWM0, 0, duty, 1000);    
	PWM_SET_CMR(PWM0, 0, u32NewCMR);
}


void PWM0_Init(void)
{
    /*
      Configure PWM0 channel 0 init period and duty(down counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = CMR / (CNR + 1)
      
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = 100 / (199 + 1) = 50%
    */
	
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, PWM_PSC - 1);

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 0, PWM_CNR);

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 0, PWM_CMR);	

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

	PWM_Start(PWM0, PWM_CH_0_MASK);

	set_flag(flag_reverse , ENABLE);
}

void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
}

void TMR1_IRQHandler(void)
{
	static uint32_t LOG = 0;
	static uint16_t CNT = 0;

	static uint16_t CNT_PWM = 1;
	static uint16_t duty = 500;	// 1 ~ 1000 , 0.1 % to 100%

    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);

		if (CNT_PWM++ == 500)
		{
			CNT_PWM = 1;
			
//			PWM_Set_Duty(duty);
			CalNewDuty(PWM0, 0, duty, 1000);

			if (is_flag_set(flag_reverse))
			{
				duty++;	
			}
			else
			{
				duty--;
			}

			if (duty == 1000)
			{
				set_flag(flag_reverse , DISABLE);				
			}
			else if (duty == 0)
			{
				set_flag(flag_reverse , ENABLE);
			}
			
			PB14 ^= 1;
		}		

		if (CNT++ >= 1000)
		{		
			CNT = 0;		
        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
		}
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

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

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
	
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk)) | SYS_GPA_MFPL_PA5MFP_PWM0_CH0;
	
    /* Update System Core Clock */
    SystemCoreClockUpdate();

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

	GPIO_Init();

	PWM0_Init();

	TIMER1_Init();
	
	
    /* Got no where to go, just loop forever */
    while(1)
    {

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
