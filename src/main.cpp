#include "board.h"
#include <stdio.h>

#include "HAL.h"
#include "temperature.h"
#include "../Marlin.h"

extern void setup();
extern void loop();
extern "C" void init();
extern "C" void store_char(unsigned char c);

union
{
    struct
    {
        char filename[11];
        uint32_t file_pos;
        uint8_t hotend;
        uint8_t hotbed;
        uint8_t chk;
    } s;
    uint16_t i[9];
}pwr_off_save; //18 bytes

uint32_t *pwr_save_addr;

void PVD_Init(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
}

#define APP_ADDRESS 0x08008000
typedef void (*pFunction)(void);
void reboot()
{
    uint32_t  JumpAddress = *(__IO uint32_t*)(APP_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;

    __disable_irq();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    __set_MSP(*(__IO uint32_t*)APP_ADDRESS);
    Jump();
}

extern "C" void PVD_IRQHandler(void)
{
    __disable_irq();
    __HAL_FLASH_DATA_CACHE_DISABLE();
    HAL_FLASH_Unlock();
    return;
}

uint16_t cpuTemp;
#ifdef MALYAN_LCD
extern void write_to_lcd_P(const char * const message);
extern void lcd_write_version();
#endif
uint8_t fan_cnt;
bool fan_on;
extern "C" void usr_tick()
{
    if ((uwTick&0x3FF)==0)
    {
        extern uint16_t AD_Value[];
        
        if (thermalManager.degTargetHotend(0)>100) {fan_on=1;WRITE(PA4, HIGH);}
        else 
        {
            float ex_temp = thermalManager.degHotend(0);
            
            if (ex_temp>100) {WRITE(PA4, HIGH);}
            else if (ex_temp<90) {WRITE(PA4, LOW);}
        }

        if (READ(X_ENABLE_PIN)==X_ENABLE_ON || 
            READ(Y_ENABLE_PIN)==Y_ENABLE_ON ||
            READ(Z_ENABLE_PIN)==Z_ENABLE_ON ||
            READ(E0_ENABLE_PIN)==E_ENABLE_ON ||
            thermalManager.degTargetBed()!=0
              )
                {WRITE(PB3, HIGH);}
        else {WRITE(PB3, LOW);}
    }
}

extern "C" void marlin()
{
    setup();
    while (1)
    {
        loop();
    }
}

extern const unsigned short ielftool_checksum;

void WR_BYTE(uint8_t val)
{
  uint8_t i = 8;
  do
  {
    if ( val & 128 )
    {
      WRITE(DOGLCD_MOSI,HIGH);
    }
    else
    {
      WRITE(DOGLCD_MOSI,LOW);
    }
    val <<= 1;
    //u8g_MicroDelay();	
    WRITE(DOGLCD_SCK, HIGH);
    //u8g_MicroDelay();	
    WRITE(DOGLCD_SCK, LOW);
    //u8g_MicroDelay();	
    i--;
  } while( i != 0 );
}

void WR_CMD(uint8_t val)
{
    WRITE(DOGLCD_A0, LOW);
    WRITE(DOGLCD_CS, LOW);
    
    WR_BYTE(val);
    
    WRITE(DOGLCD_CS, HIGH);
}

void WR_DAT(uint8_t val)
{
    WRITE(DOGLCD_A0, HIGH);
    WRITE(DOGLCD_CS, LOW);
    
    WR_BYTE(val);
    
    WRITE(DOGLCD_CS, HIGH);
}
#define WriteCommand    WR_CMD
#define WriteData       WR_DAT
void main()
{
    SCB->VTOR = 0x08008000;
    init();
    
    SET_OUTPUT(PA4);
    WRITE(PA4, HIGH);
    SET_OUTPUT(PB3);
    WRITE(PB3, HIGH);
    
#ifdef MALYAN_LCD
    
#else
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    
    SET_INPUT_PULLUP(BTN_ENC);
    SET_INPUT_PULLUP(BTN_EN1);
    SET_INPUT_PULLUP(BTN_EN2);
    
    SET_OUTPUT(DOGLCD_A0);
    SET_OUTPUT(DOGLCD_CS);
    SET_OUTPUT(DOGLCD_SCK);
    SET_OUTPUT(DOGLCD_MOSI);
    
    WRITE(DOGLCD_A0, HIGH);
    WRITE(DOGLCD_CS, HIGH);
    WRITE(DOGLCD_SCK, HIGH);
    WRITE(DOGLCD_MOSI, HIGH);
    
    WriteCommand(0xAF); //Display OFF
    WriteCommand(0xA2); //1/64 Duty 1/9 Bias
    WriteCommand(0xA0); //ADC select S0->S131
    WriteCommand(0xC8); //com1 --> com64
    WriteCommand(0x24); //
    WriteCommand(0xC8);
    WriteCommand(0x81); //Sets V0
    WriteCommand(48); //
    WriteCommand(0x2F); //voltage follower ON regulator ON booster ON
    WriteCommand(0xA6); //Normal Display (not reverse dispplay)
    WriteCommand(0xA4); //Entire Display Disable
    WriteCommand(0x40); //Set Display Start Line = com0
    WriteCommand(0xAF); //Display ON
#endif

    GPIO_InitTypeDef   GPIO_InitStructure = {0};
    GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 13, 2);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    
    GPIO_InitStructure.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 13, 1);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    marlin();
}
