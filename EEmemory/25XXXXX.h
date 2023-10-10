/*
 * 25XXXXX.h
 *
 *  Created on: 20 Sep 2021
 *      Author: Enrico
 */

#include <stdint.h>
#include <stdbool.h>


/*
 * I/O defintiions
 */
#ifdef FIRST_HW_1_0
#define EE_ON           P7OUT &= ~BIT7
#define EE_OFF          P7OUT |= BIT7
#define CS_EES_sel      P2OUT &= ~BIT3
#define CS_EES_unsel    P2OUT |= BIT3
#define WP_EES_unprot   P1OUT |= BIT0
#define WP_EES_prot     P1OUT &= ~BIT0
#else
#define EE_ON           P7OUT &= ~BIT7
#define EE_OFF          P7OUT |= BIT7
#define CS_EES_sel      P6OUT &= ~BIT2
#define CS_EES_unsel    P6OUT |= BIT2
#define WP_EES_unprot   P5OUT |= BIT3
#define WP_EES_prot     P5OUT &= ~BIT3
#endif


/*
 * specific EEPROM chip defintions
 */

#define EE_PAGE_SIZE    32
#define EE_NPAGES       128
#define EE_FIRST_ADDR   0x0000

#define EE_LAST_ADDR    ((EE_NPAGES * EE_PAGE_SIZE) + EE_FIRST_ADDR) - 1

/*
 * instruction set
 */

enum
{
    instrEE_WRSR = 1,
    instrEE_WRITE = 2,
    instrEE_READ = 3,
    instrEE_WRDI = 4,
    instrEE_RDSR = 5,
    intrEE_WREN = 6
}ee25xxxxx_en;

/*
 * data types definition
 */

typedef union
{
    struct
    {
        uint8_t lsB;
        uint8_t msB;
    }bytes;
    uint16_t Addr16;
}_eeAddrSeparate_t;

//--

typedef union
{
    struct
    {
        unsigned wip: 1;
        unsigned wel: 1;
        unsigned bp0: 1;
        unsigned bp1 :1;
        unsigned NU: 3;
        unsigned wpen :1;
    }bits;
    unsigned char val;
}_EES_sr_t;

typedef struct
    {
    unsigned int PageN;
    unsigned int FirstAddr;
    unsigned int LastAddr;
    unsigned char ReadErr;  //0:l'ultima pagina è letta correttamente; 1:l'ultima pagina letta ha dato errore di CRC / nessuna pagina caricata
    unsigned char PageData[EE_PAGE_SIZE];
    }_localPage_t;


typedef union
{
    struct
    {
        unsigned int US;    //array 0-1
        signed int SS;      //2-3
        unsigned long UL;   //4-7
        signed long SL;     //8-11
        float FL;           //12-15
        double DBL;         //16-23
    }SingleVar;
    unsigned char VarBytes[24];
}_VarSeparate_t;

typedef enum
{
    protect_none = 0,
    protect_upper_1_4 = 1,
    protect_upper_1_2 = 2,
    protect_all = 3
}_EES_protection_range;

/*
 * pages definitions
 */

#define METER_PAR_PAGE_START          0

#define USS_USER_CFG_FIRST_PAGE        100
#define USS_USER_CFG_LAST_PAGE         EE_NPAGES - 1


#define USS_USER_CFG_BEGIN  USS_USER_CFG_FIRST_PAGE * EE_PAGE_SIZE
#define USS_USER_CFG_END    (USS_USER_CFG_LAST_PAGE * EE_PAGE_SIZE) - 1
#define USS_USER_CFG_SIZE   ((USS_USER_CFG_LAST_PAGE - USS_USER_CFG_FIRST_PAGE) + 1) * EE_PAGE_SIZE
#define USS_USER_CFC_CRC_H  USS_USER_CFG_END - 1
#define USS_USER_CFC_CRC_L  USS_USER_CFG_END

/*
 * functions prototypes
 */

void Init_EES_EEPROM(void);
void EEPROM_PageRead(uint16_t PageNumber);
bool EEPROM_PageWrite(uint16_t PageNumber);
bool EEPROM_Write_Par(uint16_t PageStart, uint8_t* Data, uint16_t Size);
void EEPROM_Read_Par(uint16_t PageStart, uint8_t *Dest, uint16_t Size);
void TestPageRead(uint16_t PageNumber);
void TestPageWrite(uint16_t PageNumber);
void ReadEES_SR(void);
void WriteEES_SR(void);
bool EnableWP_EES(_EES_protection_range ProtectionRange);
void EES_EnableLatch(void);
_EES_protection_range CheckEES_protection(void);
bool EEpromCheckErrFunctionality(void);
