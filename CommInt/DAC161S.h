#ifndef __DAC161_H
#define __DAC161_H

#include <stdbool.h>
#include <stdint.h>

typedef union _UINT16
{
    struct
    {
        uint8_t lo;
        uint8_t hi;
    }byte;
    uint16_t val;
    uint8_t array[sizeof(uint16_t)];
}UINT16_t;

typedef union _INT16
{
    struct
    {
        uint8_t lo;
        uint8_t hi;
    }byte;
    int16_t val;
    uint8_t array[sizeof(int16_t)];
}INT16_t;

typedef union _UINT32
{
    struct
    {
        uint8_t lolo;
        uint8_t lohi;
        uint8_t hilo;
        uint8_t hihi;
    }byte;
    struct
    {
        UINT16_t lo;
        UINT16_t hi;
    }word;
    uint32_t val;
    uint8_t array[sizeof(uint32_t)];
} UINT32_t;

typedef union _INT32
{
    struct
    {
        uint8_t lolo;
        uint8_t lohi;
        uint8_t hilo;
        uint8_t hihi;
    }byte;
    struct
    {
        UINT16_t lo;
        UINT16_t hi;
    }word;
    int32_t val;
    uint8_t array[sizeof(int32_t)];
}INT32_t;



unsigned char OutBuffer[3];
unsigned char InBuffer[3];

unsigned char   DAC161_Err_Low_Value;
unsigned char   DAC161_Err_High_Value;
unsigned short  DAC161_Dac_Value;

float DAC161_TestOutPerc;
bool DAC161_Data_Recieved;

/*REGISTERS ADDRESSES DEFINITIONS*/
#define DAC161_XFER_REG                     0x01
#define DAC161_NOP_REG                      0x02
#define DAC161_WR_MODE_REG                  0x03
#define DAC161_DACCODE_REG                  0x04
#define DAC161_ERR_CONFIG_REG               0x05
#define DAC161_ERR_LOW_REG                  0x06
#define DAC161_ERR_HIGH_REG                 0x07
#define DAC161_RESET_REG                    0x08
#define DAC161_STATUS_REG                   0x09

#define DAC161_SPI_WRITE_CMD(address)       (address)
#define DAC161_SPI_READ_CMD(address)        (0x80 + address)

int DAC161_XFER;                         /* When PROTECT_REG_WR is set to 1, then a XFER_REG command is necessary to
                                                transfer the previous register write data into the appropriate address. Set this register
                                                to 0x00FF to perform a XFER_REG command.*/

int DAC161_NOP;                          /* No Operation. A write to this register will not change any device configuration.
                                                This command indicates that the SPI connection is functioning and is used to avoid
                                                SPI_INACTIVE errors.*/

bool DAC161_PROTECT_REG_WR;              /* 0: Register write data transfers to appropriate address immediately after CSB goes
                                                high. Default value.
                                                1: Enable protected register transfers: all register writes require a subsequent
                                                XFER_REG command to finalize the loading of register data. Refer to Optional
                                                Protected SPI Writes.*/

int DAC161_DACCODE;                      /* 16-bit natural binary word, where D15 is the MSB, which indicates the desired DAC
                                                output code.
                                                15:0 DACCODE[15:0]
                                                Note the default value of this register is based on the state of the ERR_LVL pin during
                                                startup or reset.*/

typedef struct DAC161_STATUS
{
    unsigned char DAC_RES;       /* DAC resolution
                                    On DAC161S997, returns a 111.*/

    bool ERRLVL_PIN;            /*  Returns the state of the ERRLVL pin:
                                    1 = ERRLVL pin is tied HIGH
                                    0 = ERRLVL pin is tied LOW*/

    bool FERR_STS;              /*  Frame-error status sticky bit
                                    1 = A frame error has occurred since the last STATUS read.
                                    0 = No frame error occurred since the last STATUS read.
                                    This error is cleared by reading the STATUS register. A frame error is caused by an
                                    incorrect number of clocks during a register write. A register write without an integer
                                    multiple of 24 clock cycles will cause a Frame error.*/
    bool SPI_TIMEOUT_ERR;       /*  SPI time out error
                                    1 = The SPI interface has not received a valid command within the interval set by
                                    SPI_TIMEOUT.
                                    0 = The SPI interface has received a valid command within the interval set by
                                    SPI_TIMEOUT
                                    If this error occurs, it is cleared with a properly formatted write command to a valid
                                    address.*/
    bool LOOP_STS;              /*  Loop status sticky bit
                                    1 = A loop error has occurred since last read of STATUS.
                                    0 = No loop error has occurred since last read of STATUS.
                                    Returns the loop error status. When the value in this register is 1, the DAC161S997 is
                                    unable to maintain the output current set by DACCODE at some point since the last
                                    STATUS read. This indicator clears after reading the STATUS register.*/
    bool CURR_LOOP_STS;          /*  Current loop status
                                    1 = A loop error is occurring.
                                    0 = No loop error is occurring.
                                    Returns the current Loop error status. When the value in this register is 1, the
                                    DAC161S997 is unable to maintain the output current set by DACCODE.*/
} DAC161_STATUS;

struct DAC161_Status{
    DAC161_STATUS Fields;
    uint8_t Value;
}DAC161_Status;


typedef struct DAC161_ERR_CONFIG
{
    unsigned char L_RETRY_TIME;  /* L_RETRY_TIME sets the time interval between successive attempts to reassert the
                                    desired DACCODE output current when a loop error is present. This has no effect if
                                    either MASK_LOOP_ERR is set to 1 or if DIS_RETRY_LOOP is set to 1.
                                    LOOP Retry time = (L_RETRY_TIME + 1) × 50 ms
                                    Default value = 1 (100 ms)*/

    bool DIS_RETRY_LOOP;         /* 0: When a loop error is occurring, periodically attempt to send desired DACCODE
                                    output current instead of the set ERR_LOW current. The interval between attempts is
                                    set by L_RETRY_TIMER. Default value.
                                    1: Do not periodically reassert DACCODE output when a loop error is present;
                                    reassert DACCODE after STATUS Register is read out.*/

    bool MASK_LOOP_ERR;          /* 0: When a LOOP error is detected the DAC161S997 outputs the current indicated by
                                    ERR_LOW instead of DACCCODE. Default value.
                                    1: When a Loop Error is detected the DAC161S997 tries to maintain DACCODE
                                    current on pin OUT.*/
    bool DIS_LOOP_ERR_ERRB;      /* 0: When a LOOP error is detected the DAC161S997 drives ERRB pin low. Default value.
                                    1: When a LOOP error is detected the DAC161S997 does not drive ERRB pin low.*/

    bool MASK_SPI_ERR;           /* 0: SPI timeout errors change the OUT pin current to an error value, which is
                                    determined by ERRLVL pin and contents of ERR_LOW or ERR_HIGH. Note:
                                    MASK_SPI_TOUT must be set to 0 for this to be reported. Default value.
                                    1: SPI timeout errors do not change the OUT pin current to an error value.*/

    unsigned char SPI_TIMEOUT;   /* SPI_TIMEOUT sets the time interval for SPI timeout error reporting. After each SPI
                                    write command, an internal timer is reset; if no subsequent write occurs before the
                                    timer reaches SPI timeout, a SPI timeout error is reported. SPI_ERROR reporting is
                                    inhibited by setting MASK_SPI_TOUT.
                                    A NOP write is considered a valid write and resets the timer without changing the
                                    device configuration.
                                    SPI Timeout = (SPI_TIMEOUT + 1) × 50 ms
                                    SPI_TIMEOUT default value = 1 (100 ms)*/

    bool MASK_SPI_TOUT;         /*  0: SPI timeout error reporting is enabled. A SPI timeout error drives ERRB low when a
                                    Timeout error occurs. Default value.
                                    1: SPI timeout error reporting is inhibited.*/
} DAC161_ERR_CONFIG;

struct DAC161_Err_Config{
    struct Set{
        DAC161_ERR_CONFIG Fields;
        uint16_t Value;
    }Set;
    struct Read{
        DAC161_ERR_CONFIG Fields;
        uint16_t Value;
    }Read;
}DAC161_Err_Config;

//--

typedef struct
{
    float flowLPH_4mA;
    float flowLPH_20mA;
    uint8_t reInitSeconds;
    uint8_t loopEn;
    uint8_t onlyPositiveFlow;
}_loopSet_t;

//--

// XFER_REG Register Settings - 01h
#define DAC161_XFER_REG_CMD                 0x00FF

// WR_MODE Register Settings - 03h
#define DAC161_PROTECT_REG_WR               0x0001

//  DACCODE Register Settings - 04h
#define DAC161_OUTPUT_CONVERT(uA)           (((unsigned long)uA * 2731) / 1000)

// ERR_CONFIG Register Settings - 05h
#define DAC161_L_RETRY_TIME_MASK            0x0700
#define DAC161_L_RETRY_TIME_SHIFT           8

#define DAC161_CONVERT_LOOP_RETRY_TIME(ms)  ((ms / 50) - 1)

#define DAC161_DIS_RETRY_LOOP_MASK          0x0080
#define DAC161_DISABLE_RETRY_LOOP           0x0080
#define DAC161_ENABLE_RETRY_LOOP            0x0000

#define DAC161_MASK_LOOP_ERR_MASK           0x0040
#define DAC161_LOOP_ERR_USE_ERR_LOW         0x0000
#define DAC161_LOOP_ERR_MAINTAIN_DACCODE    0x0040

#define DAC161_LOOP_ERR_ERRB_MASK           0x0020
#define DAC161_LOOP_ERR_DRIVE_ERRB          0x0000
#define DAC161_LOOP_ERR_DO_NOT_DRIVE_ERRB   0x0020

#define DAC161_SPI_ERR_MASK                 0x0010

#define DAC161_SPI_TIMEOUT_MASK             0x000e
#define DAC161_SPI_TIMEOUT_SHIFT            1
#define DAC161_CONVERT_SPI_TIMEOUT_TIME(ms) ((ms / 50) - 1)

#define DAC161_MASK_SPI_TOUT_MASK           0x0001

#define DAC161_STD_ERR_CONFIG_MASKED        (DAC161_DISABLE_RETRY_LOOP + DAC161_LOOP_ERR_MAINTAIN_DACCODE + DAC161_LOOP_ERR_DRIVE_ERRB + DAC161_SPI_ERR_MASK + DAC161_MASK_SPI_TOUT_MASK)
#define DAC161_STD_ERR_CONFIG               ((1 << DAC161_L_RETRY_TIME_SHIFT) + DAC161_ENABLE_RETRY_LOOP + DAC161_LOOP_ERR_USE_ERR_LOW + DAC161_LOOP_ERR_DRIVE_ERRB + (4 << DAC161_SPI_TIMEOUT_SHIFT) + DAC161_SPI_TIMEOUT_MASK)

// ERR_LOW Register Settings  - 06h and
// ERR_HIGH Register Settings - 07h and

#define DAC161_ERR_VALUE_MASK               0xff00
#define DAC161_ERR_VALUE_SHIFT              8
#define DAC161_CONVERT_ERR_VALUE(uA)        ((DAC161_OUTPUT_CONVERT(uA) & DAC161_ERR_VALUE_MASK))

// RESET Register Settings  - 08h
#define DAC161_RESET_VALUE                  0xc33c         // Note that a NOP command is required after this

// STATUS Register Settings  - 09h
#define DAC161_DAC_RES_MASK                 0x00e0
#define DAC161_DAC_RES_SHIFT                5

#define DAC161_ERRLVL_PIN                   0x0010

#define DAC161_FERR_STS                     0x0008

#define DAC161_SPI_TIMEOUT_ERR              0x0004

#define DAC161_LOOP_STS                     0x0002

#define DAC161_CURR_LOOP_STS                0x0001

/*************************************************************************************************************/
/*                              PROTOTYPES                                                                   */
/*************************************************************************************************************/

#ifdef __CPLUSPLUS
extern "C" {
#endif

bool DAC161_Write_Regs (unsigned short writeValues, unsigned char startReg, unsigned char lengthBytes);
bool DAC161_Read_Regs (unsigned char *readValues, unsigned char startReg, unsigned char lengthBytes);
bool DAC161_Setup  (unsigned short errConfig, unsigned short errLow_uA, unsigned short errHigh_uA);
bool DAC161_Reset (void);
bool DAC161_Nop (void);
bool DAC161_Set_Out_Value_uA (unsigned long uAmps);
bool DAC161_Set_Out_Value (unsigned short value);
bool DAC161_Set_Out_Value_Perc(float value);
unsigned char DAC161_Read_Status (void);
unsigned short DAC161_Read_DACCODE (void);
unsigned short DAC161_Read_ERR_CONFIG (void);
unsigned short DAC161_Set_ERR_CONFIG (void);
unsigned char DAC161_Read_ERR_HIGH (void);
unsigned char DAC161_Read_ERR_LOW (void);
bool DAC161_OpenMasterSPI(void);
void DAC161_CloseMasterSPI(void);
bool DAC161S_Init(void);
void loopTick(void);
bool loopHandle(void);
//bool DAC161_SPI_write (unsigned char *outData, unsigned char *inData, unsigned char length);


#ifdef __CPLUSPLUS
}
#endif

#endif
