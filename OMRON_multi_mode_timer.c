#include<stm8s.h>
int s=0,count=0,v=0,flag_timer=0,flag=0,LCD_Hold_Flag=0,pressed_reset,pressed,unpressed;
char val[6]={'0','0','.','0','0','\0'};
#define LCD_Port GPIOC->ODR			/* Define LCD data port */
#define LCD_Portb GPIOB->ODR
#define RS 4				/* Define Register Select pin */
#define EN 3				/* Define Enable signal pin */
#define inport GPIOD->IDR
#define output 3
#define startsig 5
#define resetsig 4
#define gatesig 6
#define top 5
#define eepadd 0x4001
#define onduty 2
//#define START flag_timer=1;
void _delay_ms(unsigned int x)
{
	int i,j;
  TIM4->PSCR=0x04;
	TIM4->ARR=0xFF;
	TIM4->SR1=0x00;
	TIM4->CR1=TIM4_CR1_CEN;
	for(j=0;j<(x);j++)
	{
		for(i=0;i<(4);i++)
		{
			TIM4->SR1=0x00;
			while( (0x01 & (TIM4->SR1) )==0x00);
		}
	}
}
	void _delay_us(unsigned int x)
{
	int i,j;
  TIM4->PSCR=0x00;
	TIM4->ARR=0xFF;
	TIM4->SR1=0x00;
	TIM4->CR1=TIM4_CR1_CEN;
	for(j=0;j<(x);j++)
	{
			TIM4->SR1=0x00;
			while( (0x01 & (TIM4->SR1) )==0x00);
	}
	TIM4->CR1=0x00;
}   
int Second( void )
{
	if(flag_timer==1)
	{
		int i,j;
		TIM4->PSCR=0x04;
	TIM4->ARR=0xFF;
	TIM4->SR1=0x00;
	TIM4->CR1=TIM4_CR1_CEN;
			for(i=0;i<(45);i++)
		{
			TIM4->SR1=0x00;
			while( (0x01 & (TIM4->SR1) )==0x00);
		}
		return 1;
	}
	else
	return 0;
}

void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	LCD_Portb &= ~ (0x01<<RS);		/* RS=0, command reg. */
	LCD_Port |= (0x01<<EN);		/* Enable pulse */
	_delay_us(5);
	LCD_Port &= ~ (0x01<<EN);

	_delay_us(20);

	LCD_Port =( (LCD_Port & 0x0F) | ((cmnd << 4) & 0xF0) );  /* sending lower nibble */
	LCD_Port |= (0x01<<EN);
	_delay_us(5);
	LCD_Port &= ~ (0x01<<EN);
	_delay_us(20);
}


void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	LCD_Portb |= (0x01<<RS);		/* RS=1, data reg. */
	LCD_Port|= (0x01<<EN);
	_delay_us(5);
	LCD_Port &= ~ (0x01<<EN);
	_delay_us(5);
  LCD_Port = (LCD_Port & 0x0F) | ((data << 4) & 0xF0); /* sending lower nibble */
	LCD_Port |= (0x01<<EN);
	LCD_Port &= ~ (0x01<<EN);
	_delay_us(20);
}

void LCD_Init (void)			/* LCD Initialize function */
{
				/* Make LCD port direction as o/p */
	_delay_ms(200);			/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x02);		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
	LCD_Command(0x0E);              /* Display on cursor on*/
	LCD_Command(0x06);              /* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              /* Clear display screen*/
	_delay_ms(2);
}


void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!='\0';i++)		/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_Clear(void)
{
	LCD_Command (0x01);		/* Clear display */
	_delay_ms(10);
	LCD_Command (0x80);		/* Cursor at home position */
}
void reset(void)
{
	char sal[6]={'0','0','.','0','0','\0'};
	v=0;
	count=0;
	flag_timer=0;
	LCD_Command(0x80);
	LCD_String(sal);
}
void start(void)
{
	flag_timer=1;
}
void hold(void)
{
	flag_timer=0;
}
void setop(void)
{
	((GPIOD->ODR)=(0<<output));
}
void resetop(void)
{
	(GPIOD->ODR) |=(1<<output);
}
void modeA(void)
{
	if(count>=top)
	{
		setop();
		hold();
	}
	if(flag==0)
	{
		if((!(inport&(1<<startsig)))&&(inport&(1<<resetsig))) 
		{start();
		flag=1;}
	}
	if (!(inport&(1<<resetsig)))
	{
		resetop();
		reset();
		while(!(inport&(1<<resetsig)));
		flag=0;
	}
	if(!(inport&(1<<gatesig)))
	{
		hold();
		while(!(inport&(1<<gatesig)));
		if(count!=0)
		start();
		
	}
}
void eeprom_write_byte(int value)
{
	char *address = (char *) eepadd;
	if (FLASH->IAPSR== 0x40)
    {
        FLASH->DUKR = 0xae;
        FLASH->DUKR = 0x56;
    }

        *address = (char) (value & 0xff);
				FLASH->IAPSR &= ~(1<<FLASH_IAPSR_DUL);

}
int eeprom_read_byte(void)
{
	return (*(PointerAttr uint8_t *) (MemoryAddressCast)eepadd);
}
void modeA3(void)
{
	if(flag==0)
	{
		flag=1;
		count=eeprom_read_byte();
		start();
	}
	if(count>top)
	{
		setop();
		hold();
	}
	if (!(inport&(1<<resetsig)))
	{
		resetop();
		reset();
		while(!(inport&(1<<resetsig)));
		start();
	}
	if(!(inport&(1<<gatesig)))
	{
		hold();
		while(!(inport&(1<<gatesig)));
		start();
	}
	if(!(inport&(1<<startsig)))
	{
		v=0;
		while(!(inport&(1<<startsig)));
		start();
	}
}
void LCD_Hold_Unhold(void)
{
	LCD_Hold_Flag=!LCD_Hold_Flag;
}
void modeA1(void)
{
	while(!((inport)&(1<<startsig)))
	{
		if(flag==0)
		{
			start();
			flag=1;
		}
		v =v+Second();
		LCD_Command (0x80);
			val[0]=count/10+48;
			val[1]=count%10+48;
			val[3]= (v/10)+48;
			val[4]= (v%10)+48;
			LCD_String(val);
			if(val[3]=='9'&&val[4]=='9')
			{
				v=0;
				count++;
				//eeprom_write_byte(count);
				
			}
		if(count>=top)
		{
			setop();
			hold();
		}
		if (!(inport&(1<<resetsig)))
		{
			resetop();
			reset();
			while(!(inport&(1<<resetsig)));
			start();
		}
		if(!(inport&(1<<gatesig)))
		{
			hold();
			while(!(inport&(1<<gatesig)));
			if(v!=0)
			start();
		}
		
	}
	reset();
	resetop();
	flag=0;
}
void modeA2(void)
{
	if(flag==0)
	{
		flag=1;
		start();
	}
	if(count>top)
	{
		setop();
		hold();
	}
	if (!(inport&(1<<resetsig)))
	{
		resetop();
		reset();
		while(!(inport&(1<<resetsig)));
		start();
	}
	if(!(inport&(1<<gatesig)))
	{
		hold();
		while(!(inport&(1<<gatesig)));
		if(count!=0)
		start();
	}
	if(!(inport&(1<<startsig)))
	{
		hold();
		while(!(inport&(1<<startsig)));
		if(count!=0)
		start();
	}
}
void modeB(void)
{
	if(flag==0)
	{
		if(!(inport&(1<<startsig))&&(inport&(1<<resetsig)))
		{start();
		flag=1;}
	}
	if (!(inport&(1<<resetsig)))
	{
		resetop();
		reset();
		flag=0;
	}
	if(!(inport&(1<<gatesig)))
	{
		hold();
		while(!(inport&(1<<gatesig)));
		if(count!=0)
		start();
	}
	if(count>top)
	{
		GPIOD->ODR ^=(1<<output);
		reset();
		start();
	}
}
void modeB1(void)
{
	if(flag==0)
	{
		count=eeprom_read_byte();
		if(!(inport&(1<<startsig))&&(inport&(1<<resetsig)))
		{start();
		flag=1;}
	}
		if (!(inport&(1<<resetsig)))
		{
			resetop();
			reset();
			while((inport&(1<<startsig)));
			start();
		}
		if(!(inport&(1<<gatesig)))
		{
			hold();
			while(!(inport&(1<<gatesig)));
			if(count!=0)
			start();
		}
		if(count>top)
		{
			GPIOD->ODR ^=(1<<output);
			reset();
			start();
		}
}
void modeD(void)
{
	
		if(!(inport&(1<<resetsig)))
		{
			pressed++;
			reset();
			if(pressed>10)
			{
				pressed=0;
			  resetop();
			}
		}
		if(!(inport&(1<<startsig)))
		{
			setop();
			reset();
			while(!(inport&(1<<startsig)))
			{
					if(!(inport&(1<<resetsig)))
					{pressed++;
					if(pressed>100)
					{
						resetop();
						while(!(inport&(1<<resetsig)));
						pressed=0;
						setop();
					}
				  }
			}
			if((inport&(1<<resetsig)))
			{
				start();
			}
		}
			if(count>top)
			{
				resetop();
				reset();
			}
			if(!(inport&(1<<gatesig)))
			{
				hold();
				while(!(inport&(1<<gatesig)));
				if(count!=0)
				start();
				
			}
}
void modeE(void)
{
	if(flag==0)
	{
		if(!(inport&(1<<startsig)))
		{setop();
			reset();
		start();
		flag=1;}
	}
	if((inport&(1<<startsig)))
	{
		flag=0;
	}
		if (!(inport&(1<<resetsig)))
		{
			resetop();
			reset();
			while(!(inport&(1<<resetsig)));
			if(!(inport&(1<<startsig)))
			{
				start();
				setop();
			}
		}
		if(!(inport&(1<<gatesig)))
		{
			hold();
			while(!(inport&(1<<gatesig)));
			if(count!=0)
			start();
		}
	if(count>top)
	{
		reset();
		resetop();
	}
}
void modeF(void)
{
	if(flag==0)
	{
		flag=1;
		count=eeprom_read_byte();
		LCD_Command (0x80);
			val[0]=count/10+48;
			val[1]=count%10+48;
			val[3]= (v/10)+48;
			val[4]= (v%10)+48;
			LCD_String(val);
	}
	if(!(inport&(1<<startsig)))
	{
		start();
	}
		while(!(inport&(1<<startsig)))
		{
			v =v+Second();
		LCD_Command (0x80);
			val[0]=count/10+48;
			val[1]=count%10+48;
			val[3]= (v/10)+48;
			val[4]= (v%10)+48;
			LCD_String(val);
			if(val[3]=='9'&&val[4]=='9')
			{
				v=0;
				count++;
				eeprom_write_byte(count);
				
			}

			if (!(inport&(1<<resetsig)))
			{
				resetop();
				reset();
				while(!(inport&(1<<startsig)));
				if(!(inport&(1<<startsig)))
				start();
			}
			if(!(inport&(1<<gatesig)))
			{
				hold();
				while(!(inport&(1<<gatesig)));
				if(v!=0)
				start();
			}
			if(count>top)
			{
				setop();
				hold();
			}
	 }
		hold();
		if(!(inport&(1<<gatesig)))
		{
			hold();
		}
		if (!(inport&(1<<resetsig)))
		{
			resetop();
			reset();
		}
		if(count>top)
		{
			setop();
			hold();
		}
}
void modeZ(void)
{
	if(flag==0)
	{
		if(!(inport&(1<<startsig))&&(inport&(1<<resetsig)))
		{
			start();
			setop();
			flag=1;
		}
	}
	if(count>onduty)
	{
		resetop();
	}
	if(count>top)
	{
		setop();
		reset();
		start();
	}
	if (!(inport&(1<<resetsig)))
	{
		resetop();
		reset();
		flag=0;
	}
	if(!(inport&(1<<gatesig)))
	{
		hold();
		while(!(inport&(1<<gatesig)));
		if(count!=0)
		start();
	}
}
void modeS(void)
{
	if(flag==0)
	{
		if(!(inport&(1<<startsig)))
		{
			pressed++;
			if(pressed==1)
			{while(!(inport&(1<<resetsig))||!(inport&(1<<gatesig)));
			if(	TIM4->CR1==0x01)
			{
				hold();
			}
			else
			{
				start();
			}
			flag=1;
		}
		pressed=0;}
	}
	if((inport&(1<<startsig)))
	{
		unpressed++;
		if(unpressed==1)
		{
			flag=0;
			unpressed=0;
		}
	}
	if (!(inport&(1<<resetsig))||!(inport&(1<<gatesig)))
	{
		pressed_reset++;
		if(pressed_reset==1)
		{if(v==0)
		{
			reset();
			resetop();
			flag=0;
		}
		else
		{
			LCD_Hold_Unhold();
		}
	}
		pressed_reset=0;
	}
	if(count>top)
	{
		setop();
		hold();
	}
}

void main()
{
	int i;
pressed=0;
	unpressed=0;
	pressed_reset=0;
flag_timer=0;
flag=0;
	count=0;
	v=0;
	CLK->CMSR=CLK_CMSR_RESET_VALUE;
		CLK->SWR=CLK_SWR_RESET_VALUE;
		CLK->CKDIVR=0x00;
	GPIOC->DDR=0xFF;
	GPIOC->CR1=0xFF;
	GPIOB->DDR|=(1<<RS);
	GPIOB->CR1|=(1<<RS);
	GPIOD->DDR =0x0F;
	GPIOD->CR1=0xFF;
	GPIOD->CR2=0xFF;
	GPIOD->ODR=0xFF;
	
	LCD_Init();
	while (1)
	{
		modeA();
		v =v+Second();
			LCD_Command (0x80);
			val[0]=count/10+48;
			val[1]=count%10+48;
			val[3]= (v/10)+48;
			val[4]= (v%10)+48;
			if(LCD_Hold_Flag==0)
			LCD_String(val);
			if(v==99)
			{
				v=0;
				count++;
				//eeprom_write_byte(count);
				
			}
			
	}
}