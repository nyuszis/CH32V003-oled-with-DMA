/**
 * file created 2023-12-18 
 * by Péter Támcsu 
 * 
 * 
 *   CH32V003
 *   PD6   RX / A6
 *   PD5   TX / A5
 * 
*/

#include <stdio.h>

#include "ch32v003fun.h" // for register definitions and SystemInit and UART functions

#define SYSTICK_SR_CNTIF (1<<0)
#define SYSTICK_CTLR_STE (1<<0)
#define SYSTICK_CTLR_STIE (1<<1)
#define SYSTICK_CTLR_STCLK (1<<2)
#define SYSTICK_CTLR_STRE (1<<3)
#define SYSTICK_CTLR_SWIE (1<<31)

volatile u32 systick_cnt;
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
    // next interrupt in 1ms
	SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK/1000);
	// clear IRQ 
	SysTick->SR = 0;
	// increment counter
	systick_cnt++;
}


void delay_ms(u32 ms)
{
    u32 start = systick_cnt;
    while (systick_cnt - start < ms);
}


/**
 * i2c
*/

// I2C Parameters
#define I2C_CLKRATE   400000    // I2C bus clock rate (Hz)
#define I2C_MAP       0         // I2C pin mapping (see above)

// I2C_init()               Init I2C with defined clock rate (400kHz)
// I2C_start(addr)          I2C start transmission, addr must contain R/W bit
// I2C_write(b)             I2C transmit one data byte via I2C
// I2C_stop()               I2C stop transmission

// I2C pin mapping (set below in I2C parameters):
// ----------------------------------------------
// I2C_MAP    0     1     2
// SDA-pin   PC1   PD0   PC6
// SCL-pin   PC2   PD1   PC5 

#define I2C_CLKRATE   400000    // I2C bus clock rate (Hz)
#define I2C_MAP       0         // I2C pin mapping (see above)

// I2C event flag definitions
#define I2C_START_GENERATED     0x00010003    // BUSY, MSL, SB
#define I2C_ADDR_TRANSMITTED    0x00820003    // BUSY, MSL, ADDR, TXE
#define I2C_BYTE_TRANSMITTED    0x00840003    // BUSY, MSL, BTF, TXE
#define I2C_checkEvent(n)       (((((uint32_t)I2C1->STAR1<<16) | I2C1->STAR2) & n) == n)

#define SET_OUTPP( PORT, PIN ) PORT->CFGLR &= ~(GPIO_CFGLR_CNF##PIN); PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*PIN)

// I2C Timeout count
#define TIMEOUT_MAX 100000

// Start I2C transmission (addr must contain R/W bit)
void I2C_start(u8 addr) {
  s32 timeout ;
  timeout = TIMEOUT_MAX;
  while(  (I2C1->STAR2 & I2C_STAR2_BUSY) && timeout-- );            // wait until bus ready
  if (timeout==-1) return;
  I2C1->CTLR1 |= I2C_CTLR1_START;                 // set START condition
  timeout = TIMEOUT_MAX;
  while(!(I2C1->STAR1 & I2C_STAR1_SB) && timeout-- );           // wait for START generated
  if (timeout==-1) return;
  I2C1->DATAR = addr;   
  timeout = TIMEOUT_MAX;                          // send slave address + R/W bit
  while( !I2C_checkEvent(I2C_ADDR_TRANSMITTED) && timeout--);   // wait for address transmitted
  if (timeout==-1) return;
}

// Send data byte via I2C bus
void I2C_write(u8 data) {
  s32 timeout ;
  timeout = TIMEOUT_MAX;

  while(!(I2C1->STAR1 & I2C_STAR1_TXE) && timeout-- );          // wait for last byte transmitted
  if (timeout==-1) return;
  I2C1->DATAR = data;                             // send data byte
}

// Stop I2C transmission
void I2C_stop(void) {
  s32 timeout ;
  timeout = TIMEOUT_MAX;

  while(!(I2C1->STAR1 & I2C_STAR1_BTF) && timeout-- );          // wait for last byte transmitted
  if (timeout==-1) return;
  I2C1->CTLR1 |= I2C_CTLR1_STOP;                  // set STOP condition
}

#define SSD1306_I2C_ADDR 0x3c
// F_CPU    = 24000000


/**
 * Numbers for 128x32 OLED
 * by https://oleddisplay.squix.ch/
*/
const u8 nums[10][56] = {
    {0x00,0x00,0x00,0x00,0x00,0xF0,0x07,0x00,0x00,0xFE,0x3F,0x00,0x80,0xFF,0xFF,0x00,0x80,0xFF,0xFF,0x00,0xC0,0x03,0xE0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x03,0xE0,0x01,0x80,0xFF,0xFF,0x00,0x80,0xFF,0xFF,0x00,0x00,0xFE,0x3F,0x00,0x00,0xE0,0x03,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x07,0x00,0x00,0x80,0x03,0x00,0x00,0x80,0xFF,0xFF,0x01,0xC0,0xFF,0xFF,0x01,0xC0,0xFF,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x80,0x01,0x00,0x0F,0xC0,0x01,0x80,0x0F,0xE0,0x01,0x80,0x07,0xF0,0x01,0xC0,0x01,0xFC,0x01,0xC0,0x01,0xDE,0x01,0xC0,0x01,0xCF,0x01,0xC0,0x81,0xC7,0x01,0xC0,0xC3,0xC3,0x01,0x80,0xFF,0xC1,0x01,0x80,0xFF,0xC0,0x01,0x00,0x3F,0xC0,0x01,0x00,0x00,0xC0,0x01,},
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x18,0x00,0x00,0x0F,0x78,0x00,0x80,0x0F,0xF8,0x00,0x80,0x07,0xE0,0x00,0xC0,0xE1,0xC0,0x01,0xC0,0xE1,0xC0,0x01,0xC0,0xE1,0xC0,0x01,0xC0,0xE1,0xC0,0x01,0xC0,0xF3,0xE1,0x01,0x80,0xBF,0xFF,0x00,0x80,0xBF,0xFF,0x00,0x00,0x1F,0x3F,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x00,0x00,0x00,0x1F,0x00,0x00,0xC0,0x1F,0x00,0x00,0xE0,0x1D,0x00,0x00,0x78,0x1C,0x00,0x00,0x3E,0x1C,0x00,0x00,0x0F,0x1C,0x00,0xC0,0x07,0x1C,0x00,0xC0,0xFF,0xFF,0x01,0xC0,0xFF,0xFF,0x01,0xC0,0xFF,0xFF,0x01,0x00,0x00,0x1C,0x00,0x00,0x00,0x1C,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x38,0x00,0xC0,0xFF,0x78,0x00,0xC0,0xFF,0xF8,0x00,0xC0,0xEF,0xE0,0x01,0xC0,0x71,0xC0,0x01,0xC0,0x71,0xC0,0x01,0xC0,0x71,0xC0,0x01,0xC0,0xF1,0xE0,0x01,0xC0,0xF1,0xF1,0x00,0xC0,0xE1,0xFF,0x00,0xC0,0xC1,0x7F,0x00,0x00,0x00,0x1F,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x3F,0x00,0x00,0xFE,0x7F,0x00,0x00,0xFF,0xFF,0x00,0x80,0xE7,0xE0,0x01,0x80,0x73,0xC0,0x01,0xC0,0x73,0xC0,0x01,0xC0,0x71,0xC0,0x01,0xC0,0xF1,0xE0,0x01,0xC0,0xF1,0xF1,0x00,0x00,0xE0,0xFF,0x00,0x00,0xC0,0x7F,0x00,0x00,0x00,0x1F,0x00,},
    {0x00,0x00,0x00,0x00,0xC0,0x01,0x00,0x00,0xC0,0x01,0x00,0x00,0xC0,0x01,0x00,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xF0,0x01,0xC0,0x01,0xFC,0x01,0xC0,0x81,0x3F,0x00,0xC0,0xE1,0x0F,0x00,0xC0,0xF9,0x03,0x00,0xC0,0x7F,0x00,0x00,0xC0,0x1F,0x00,0x00,0xC0,0x07,0x00,0x00,0xC0,0x01,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x00,0x00,0x1F,0x7F,0x00,0x80,0xBF,0xFF,0x00,0x80,0xFF,0xFF,0x00,0xC0,0xF3,0xE1,0x01,0xC0,0xE1,0xC0,0x01,0xC0,0xE1,0xC0,0x01,0xC0,0xE1,0xC0,0x01,0xC0,0xF3,0xE1,0x01,0x80,0xFF,0xFF,0x00,0x80,0xBF,0xFF,0x00,0x00,0x1F,0x7F,0x00,0x00,0x00,0x08,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0xFF,0x01,0x00,0x80,0xFF,0x03,0x00,0x80,0xC7,0xC7,0x01,0xC0,0x83,0xC7,0x01,0xC0,0x01,0xC7,0x01,0xC0,0x01,0xE7,0x01,0xC0,0x01,0xE7,0x00,0xC0,0x83,0xF3,0x00,0x80,0xFF,0x7F,0x00,0x00,0xFF,0x3F,0x00,0x00,0xFE,0x0F,0x00,0x00,0x00,0x00,0x00,}
};

enum dmai2cstates { DIS_IDLE, DIS_BUSY } ;
volatile enum dmai2cstates DMAI2CSTATE = DIS_IDLE;

/**
 * DMA1 ch6 interrupt handler
 * I2C DMA complete after sending stop and set DMAI2CSTATE to DIS_IDLE
*/
void DMA1_Channel6_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel6_IRQHandler(void) {
    DMA1_Channel6->CFGR &= ~DMA_CFGR1_EN; // Disable DMA1_Channel6
    DMA1->INTFCR = DMA1_FLAG_GL6; // TCTIF6 Clear the global interrupt flag for channel6    
    // I2C_stop(); // set STOP condition 
    I2C1->CTLR1 |= I2C_CTLR1_STOP; // set STOP condition
    DMAI2CSTATE = DIS_IDLE;
}


u16 adc_get( void )
{
    /**
     * get ADC value
    */
	// start sw conversion (auto clears)
	ADC1->CTLR2 |= ADC_SWSTART;
	
	// wait for conversion complete
	while(!(ADC1->STATR & ADC_EOC));
	
	// get result
	return ADC1->RDATAR;
}

const u8 init128x32[] = { 0xAe, // disp off
             0xD5, // clk div
             0x80, // suggested ratio
             0xA8, 0x1F, // set multiplex: 0x1F for 128x32, 0x3F for 128x64
             0xD3,0x0, // display offset
             0x40, // start line
             0x8D,0x14,  // charge pump
            //  0x20,0x12, // memory mode - page mode
             0x20,0x01, // memory mode - vertical mode
             0xA1, // seg remap 1 
             0xC8, // comscandec
             
             0xDA,0x02, // set compins - use 0x12 for 128x64 and 0x02 for 128x32 display
             
             0x81,0xCF, // set contrast
             0xD9,0xF1, // set precharge
             0xDb,0x40, // set vcom detect
             0xA4, // display all on
             0xA6, // display normal (non-inverted)
             0xAf   // disp on
        };


static inline void setrect( u8 x1, u8 y1, u8 x2, u8 y2 ) {
    /**
     * ssd1306 set rect for drawing
    */
    I2C_start(SSD1306_I2C_ADDR<<1);
    I2C_write( 0x00 );
    I2C_write( 0x21 ); I2C_write( x1 ); I2C_write(x2 ); // set column address
    I2C_write( 0x22 ); I2C_write( y1 ); I2C_write( y2 ); // set page address
    I2C_stop();
}

#define SET_INANALOG( PORT, PIN ) PORT->CFGLR &= ~(GPIO_CFGLR_CNF##PIN)

volatile u8 buf[10]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

u8 waitI2CDMA( void ) {
    /**
     * wait for DMA1_Channel6 transfer complete
     * not used
    */
    s32 timeout ;
    timeout = TIMEOUT_MAX;
    while (!(DMA1->INTFR & DMA1_FLAG_TC6) && timeout--);  // wait for DMA1_Channel6 transfer complete
    return (timeout==-1) ? 0 : 1;
}

void fill( u32 n, u8 b) {
    /**
     * fill n bytes with b
     * with DMA
    */
    DMA1_Channel6->CFGR &= ~(DMA_MemoryInc_Enable); // no increment for memory
    DMA1_Channel6->CNTR = n; // set count
    DMA1_Channel6->MADDR = (u32)&b; // set target for DMA1 to buf
    I2C_start(SSD1306_I2C_ADDR<<1);
    I2C_write( 0x40 );
    DMA1_Channel6->CFGR |= DMA_CFGR1_EN; // Enable DMA1_Channel6
}

void draw( const u8 *b, u32 n ) {
    /**
     * draw n bytes from b to screen
     * with DMA
    */
    DMAI2CSTATE = DIS_BUSY;
    DMA1_Channel6->CFGR |= DMA_MemoryInc_Enable; // increment for memory
    DMA1_Channel6->CNTR = n; // set count
    DMA1_Channel6->MADDR = (u32)b; // set target for DMA1 to buf
    I2C_start(SSD1306_I2C_ADDR<<1);
    I2C_write( 0x40 );
    DMA1_Channel6->CFGR |= DMA_CFGR1_EN; // Enable DMA1_Channel6
}

void drawnum( u8 c, u16 pos ) {
    /**
     * draw number at pos
    */
    if (pos>6) return;
    if (c==' ') ;
    else if (c>='0' && c<='9') ;
    else return;
    
    u8 n = c-'0';
    u8 xpos = 16*pos + 2;
    setrect( xpos, 0, xpos+14, 3 );

    if (c==' ') fill( 4*14, 0x00 );
    else draw( nums[n] , 4*14 );
}    

enum digitstate { DS_IDLE, DS_START, DS_0, DS_1 , DS_2 , DS_3, DS_STOP } ;


int main(void) {
    SystemInit(); // initialize the system clock and USART to 115200 8N1, only TX enabled
    
    Delay_Ms( 100 );    
    SysTick->CTLR = 0;
    NVIC_EnableIRQ(SysTicK_IRQn);    
	SysTick->CMP = (FUNCONF_SYSTEM_CORE_CLOCK/1000)-1;
	SysTick->CNT = 0;
	systick_cnt = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |SYSTICK_CTLR_STCLK;        
    RCC->AHBPCENR |= RCC_DMA1EN; // Enable DMA1 peripheral clock
    RCC->APB2PCENR |= RCC_IOPCEN;  // Enable GPIOC peripheral clock
    

    SET_OUTPP( GPIOC, 3 ); // PC3 state LED
    SET_INANALOG( GPIOC, 4 ); // PC41 ADC2 input 
    
    // GPIOC->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
    
    if (1) {
        
        // RCC->CFGR0 &= ~(0x1F<<11);
        RCC->APB2PCENR |= RCC_APB2Periph_ADC1;
         
         // Reset ADC1 to init all regs
        RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
        RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1; 
        
        RCC->CFGR0 &= ~RCC_ADCPRE;  // Clear out the bis in case they were set
        RCC->CFGR0 |= RCC_ADCPRE_DIV2;	// set it to 010xx for /2. 

	    
        ADC1->SAMPTR2 &= ~(ADC_SMP0<<(3*2)); // CH 2
	    ADC1->SAMPTR2 |= 0b111 <<(3*2);	// 0:7 => 3/9/15/30/43/57/73/241 cycles        
        
        ADC1->RSQR3 = (2 << 0);

        ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;
        
        ADC1->CTLR2 |= ADC_RSTCAL;
	    while(ADC1->CTLR2 & ADC_RSTCAL);
	
	    // Calibrate
	    ADC1->CTLR2 |= ADC_CAL;
	    while(ADC1->CTLR2 & ADC_CAL);        

    }   

    if (1) {
        /**
         * I2C init 
         * pins
         * PC1 - SDA
         * PC2 - SCL
         * config
         * 
        */
        RCC->APB2PCENR |= RCC_IOPCEN;  // Enable GPIOC peripheral clock
        // PC1 is SDA, 10MHz Output, alt func, open-drain
        GPIOC->CFGLR &= ~(0xf<<(4*1));
        GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*1);
        
        // PC2 is SCL, 10MHz Output, alt func, open-drain
        GPIOC->CFGLR &= ~(0xf<<(4*2));
        GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*2);        

        RCC->APB1PCENR |= RCC_I2C1EN; // Enable I2C1 peripheral clock
        I2C1->CTLR2 = 4; // Set peripheral clock frequency in MHz

        // Set bus clock configuration
        I2C1->CKCFGR = (FUNCONF_SYSTEM_CORE_CLOCK / (3 * I2C_CLKRATE)) | I2C_CKCFGR_FS;
        
        // I2C1->CTLR2 |= I2C_CTLR2_LAST; // This bit is used in master receiver mode to permit the generation of a NACK on the last received data
        
        // I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN; // Event interrupt enable bit. Set this bit to enable event interrupt. This interrupt will be generated under the following conditions.
        I2C1->CTLR2 |= I2C_CTLR2_DMAEN;

        // Enable I2C
        I2C1->CTLR1 = I2C_CTLR1_PE;         

    }
    
    if (1) {
        // SSD1306 OLED INIT
        
        I2C_start(SSD1306_I2C_ADDR<<1);
        I2C_write( 0x00 ); // command
        for (u32 i=0; i<sizeof(init128x32); i++) {
            I2C_write(init128x32[i]);
        }
        I2C_stop();

        if (0) {
            I2C_start(SSD1306_I2C_ADDR<<1);
            I2C_write( 0x00 ); // command
            I2C_write( 0x21 ); // set column address
            I2C_write( 0 ); // start
            I2C_write( 127 ); // end
            I2C_write( 0x22 ); // set page address
            I2C_write( 0 ); // start
            I2C_write( 3 ); // end
            I2C_stop();

            I2C_start(SSD1306_I2C_ADDR<<1);
            I2C_write( 0x40 ); // data
            for (u32 i=0; i<128*4; i++) {
                I2C_write(0x00);
            }
            I2C_stop();
        }


    }
    

    if (1) {
        /**
         *  i2c DMA with interrupt
        */
        RCC->AHBPCENR |= RCC_DMA1EN; // Enable DMA1 peripheral clock
        DMA1_Channel6->PADDR = (u32)&I2C1->DATAR;
        // set target for DMA1 to buf
	    DMA1_Channel6->MADDR = (u32)buf;
	    DMA1_Channel6->CFGR  =
            DMA_M2M_Disable |		 
            // DMA_Priority_VeryHigh |
            DMA_Priority_High |
            DMA_MemoryDataSize_Byte |
            DMA_PeripheralDataSize_Byte |
            DMA_MemoryInc_Enable |
            // DMA_Mode_Circular |
            DMA_DIR_PeripheralDST; // Data transfer direction. 1: Read from memory. 0: Read from peripheral.
	    
        DMA1_Channel6->CFGR |= DMA_CFGR1_TCIE; // Enable DMA1_Channel6 transfer complete interrupt 
        NVIC_EnableIRQ( DMA1_Channel6_IRQn );
        
    }
    
    if (1) {
        /**
         * clear screen
        */
        setrect( 0, 0, 127, 3 );
        fill( 128*4, 0x00 );
    }

    u32 T_IDLE = 0;
    u32 T_DISP = 0;
    u8 digits[4] = {' ',' ',' ',' '};
    enum digitstate ds = DS_IDLE;
    // unsigned char digits[]="   0";
    while (1) {
        
        if (systick_cnt - T_DISP > 50) {
            T_DISP = systick_cnt;
            u32 a = 0;
            for (u8 i=0; i<8; i++) a+=adc_get();
            a /= 8;
            
            digits[0]= ( a/1000 >0) ? '0' + (( a/1000 )%10) : ' ';
            digits[1]= ( a/100 >0)  ? '0' + (( a/100 )%10) : ' ';
            digits[2]= ( a/10 >0)   ? '0' + (( a/10 )%10) : ' ';
            digits[3]= '0' + (a%10);
            ds = DS_START;
        }
        
        if (ds >= DS_START && ds <= DS_3 && DMAI2CSTATE==DIS_IDLE ) {
            ds++; // next state
            // waitI2CDMA(); // moved to if
            if (ds == DS_STOP) {
                ds = DS_IDLE;
            } else {
                drawnum( digits[ds - DS_0], ds - DS_0 );
            }
        }  

        if (systick_cnt - T_IDLE > 100) {
            T_IDLE = systick_cnt;
            GPIOC->OUTDR ^= (1<<3);            
        }
    }



}