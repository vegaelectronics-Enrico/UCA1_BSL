/*
 * DAC161S.c
 *
 *  Created on: 12 Aug 2022
 *      Author: Enrico
 */

#include "DAC161S.h"
#include "CommInterfaces.h"
#include "hal.h"


uint8_t DAC161_RcvData[3];
uint8_t LoopReInitCnt = 0;
bool doReInit = false;
bool doUpdate = false;

#warning "assorbimento da batteria non misurato e non aggiornato sul calcolo con 4-20 attiva!!!!"

/*************************************************************************************************************************************************
 *  DAC161_Write_Regs
 **************************************************************************************************************************************************/
/*!
 * @brief Writes registers on the DAC161S997.
 *
 * This function will execute a write register command to the DAC161S997. This function can be used to update one or more registers on the DAC161S997.
 * No error checking is performed, so it is the user's responsibility to make sure they do not attempt to write past the end of the DAC161S997 registers.
 *
 * @param[out]    writeValues    values to place in the DAC161
 * @param[in]     startReg       Address of the first register to write
 * @param[in]     length         Number of registers to write.
 *
 * @return  None
 *
 **************************************************************************************************************************************************/
bool DAC161_Write_Regs (unsigned short writeValues, unsigned char startReg, unsigned char lengthBytes)
{
    //unsigned char outData[3];
    OutBuffer[0] = DAC161_SPI_WRITE_CMD(startReg);

    // Switch Endianess
    OutBuffer[1] = (unsigned char)((writeValues & 0xFF00) >> 8);
    OutBuffer[2] = (unsigned char)(writeValues & 0x00FF);
    if( UCB1_SPI_SendReceiveBytes_420 (OutBuffer, DAC161_RcvData, lengthBytes+1) )    // Add 1 to length for command byte
        return true;
    else
        return false;
}

/*************************************************************************************************************************************************
 *  DAC161_Read_Regs
 **************************************************************************************************************************************************/
/*!
 * @brief Reads registers on the DAC161S997.
 *
 * This function will execute a read register command to the DAC161S997 and return the resultant data. This function can be used to read one or more
 * registers from the DAC161S997. No error checking is performed, so it is the user's responsibility to make sure they do not attempt to read past
 * the end of the DAC161S997 registers.
 *
 * @param[out]   *readValues     Pointer to place the 8 bit register values from the DAC161
 * @param[in]     startReg       Address of the first register to read
 * @param[in]     length         Number of registers to read.
 *
 * @return  None
 *
 **************************************************************************************************************************************************/
bool DAC161_Read_Regs (unsigned char *readValues, unsigned char startReg, unsigned char lengthBytes)
{
    OutBuffer[0] = 0x55;
    OutBuffer[1] = 0x55;
    OutBuffer[2] = 0x55;

    // Sets the address for the read command
    OutBuffer[0] = DAC161_SPI_READ_CMD(startReg);
    if( UCB1_SPI_SendReceiveBytes_420 (OutBuffer, readValues, lengthBytes+1) )    // Add 1 to length for command byte
    {
        // Performs actual read of previous address
        OutBuffer[0] = DAC161_NOP_REG;
         if( UCB1_SPI_SendReceiveBytes_420 (OutBuffer, readValues, lengthBytes+1) )    // Add 1 to length for command byte
             return true;
         else
             return false;
    }
    else
        return false;
}

/*************************************************************************************************************************************************
 *  Setup_DAC161S997
 **************************************************************************************************************************************************/
/*!
 * @brief Performs the setup of the DAC161S997.
 *
 * This function will configure the DAC161S997.
 *
 * @param[in]     errConfig       Error Configuration for the DAC161S997 (DAC161_STD_ERR_CONFIG)
 * @param[in]     errLow_uA       Output level (micro Amps) for a Low Error
 * @param[in]     errHigh_uA      Output level (micro Amps) for a High Error
 *
 * @return  None
 *
 **************************************************************************************************************************************************/
bool DAC161_Setup (unsigned short errConfig, unsigned short errLow_uA, unsigned short errHigh_uA)
{
    unsigned short errValue;

    if( DAC161_Write_Regs (errConfig, DAC161_ERR_CONFIG_REG, 2) )
    {
         errValue = DAC161_CONVERT_ERR_VALUE(errLow_uA) & 0x7f00;
         if( DAC161_Write_Regs (errValue, DAC161_ERR_LOW_REG, 2) )
         {
             errValue = DAC161_CONVERT_ERR_VALUE(errHigh_uA) & 0xff00;
             if (errValue < 0x80)
                 errValue = 0x80;
             if( DAC161_Write_Regs (errValue, DAC161_ERR_HIGH_REG, 2) )
                 return true;
             else
                 return false;
         }
         return false;
    }
    else
        return false;
}

/*************************************************************************************************************************************************
 *  DAC161_Reset
 **************************************************************************************************************************************************/
/*!
 * @brief Sends a Reset Command to the DAC161S997.
 *
 * This function sends a Reset command to the DAC161S997 on the SPI bus.
 *
 * @return  None
 *
 **************************************************************************************************************************************************/
bool DAC161_Reset (void)
{
    unsigned short resetCode = DAC161_RESET_VALUE;

    if( DAC161_Write_Regs (resetCode, DAC161_RESET_REG, 2) )
    {
        if( DAC161_Write_Regs (resetCode, DAC161_NOP_REG, 2) )          // Nop required after reset
            return true;
        else
            return false;
    }
    return false;
}

/*************************************************************************************************************************************************
 *  DAC161_Nop
 **************************************************************************************************************************************************/
/*!
 * @brief Sends a Nop Command to the DAC161.
 *
 * This function sends a Nop command to the DAC161S997 on the SPI bus. The DAC161 will timeout and move into an error state if it does not receive
 * regular commands for the SPI master. This command can be used to notify the DAC161S997 that the system is still operational, but no change to the
 * DAC161S997 is desired.
 *
 * @return  None
 *
 **************************************************************************************************************************************************/
bool DAC161_Nop (void)
{
    unsigned short deadCode = 0xdead;

    if( DAC161_Write_Regs (deadCode, DAC161_NOP_REG, 2) )
        return true;
    else
        return false;
}

/*************************************************************************************************************************************************
 *  DAC161_Set_Out_Value_uA
 **************************************************************************************************************************************************/
/*!
 * @brief Sets the output current from the DAC161S997.
 *
 * The DAC161S997 is designed to be used in a 4-20mA system. This function sets the desired output current.
 *
 * @param[in]     uAmps         Value in uAmps to output from the DAC161S997
 *
 * @return  None
 *
 **************************************************************************************************************************************************/
bool DAC161_Set_Out_Value_uA (unsigned long uAmps)
{
    unsigned short value = DAC161_OUTPUT_CONVERT(uAmps);

    if( DAC161_Write_Regs (value, DAC161_DACCODE_REG, 2) )
        return true;
    else
        return false;
}

/*************************************************************************************************************************************************
 *  DAC161_Set_Out_Value
 **************************************************************************************************************************************************/
/*!
 * @brief Sets the output from the DAC161S997.
 *
 * The DAC161S997 is designed to be used in a 4-20mAmp system. Data is communicated by changing the current output from the DAC161S997.
 * This function sets the desired output current.
 *
 * @param[in]     value         16 Bit Hex value to output from the DAC161S997
 *
 * @return  None
 *
 **************************************************************************************************************************************************/
bool DAC161_Set_Out_Value (unsigned short value)
{
    if(DAC161_Write_Regs (value, DAC161_DACCODE_REG, 2))
        return true;
    else
        return false;
}

/*************************************************************************************************************************************************
*  DAC161_Read_Status
**************************************************************************************************************************************************/
/*!
* @brief Reads thE status register from the DAC161.
*
* This function returns the current value in the status register of the DAC161
*
* @return  DAC161 Status     (DAC161_FERR_STS, DAC161_SPI_TIMEOUT_ERR, DAC161_LOOP_STS, DAC161_CURR_LOOP_STS)
**************************************************************************************************************************************************/
unsigned char DAC161_Read_Status (void)
{
    unsigned char returnValue[3];

    DAC161_Status.Value = 0;

    if( DAC161_Read_Regs (returnValue, DAC161_STATUS_REG, 2) )
    {
        DAC161_Status.Value                    = returnValue[2];
        DAC161_Status.Fields.DAC_RES           = (DAC161_Status.Value & DAC161_DAC_RES_MASK) >> DAC161_DAC_RES_SHIFT;
        DAC161_Status.Fields.ERRLVL_PIN        = DAC161_Status.Value & DAC161_ERRLVL_PIN;
        DAC161_Status.Fields.FERR_STS          = DAC161_Status.Value & DAC161_FERR_STS;
        DAC161_Status.Fields.SPI_TIMEOUT_ERR   = DAC161_Status.Value & DAC161_SPI_TIMEOUT_ERR;
        DAC161_Status.Fields.LOOP_STS          = DAC161_Status.Value & DAC161_LOOP_STS;
        DAC161_Status.Fields.CURR_LOOP_STS     = DAC161_Status.Value & DAC161_CURR_LOOP_STS;

        if(DAC161_Status.Fields.DAC_RES != 7)
            DAC161_Status.Value = 0xFF;
    }
    else
    {
        DAC161_Status.Value = 0xFF;
    }
    return DAC161_Status.Value;
}

/*************************************************************************************************************************************************
*  DAC161_Read_DACCODE
**************************************************************************************************************************************************/
/*!
* @brief Reads thE DACCODE register from the DAC161.
*
* This function returns the current value in the DACCODE register of the DAC161
*
* @return  DACCODE
**************************************************************************************************************************************************/
unsigned short DAC161_Read_DACCODE (void)
{
    unsigned char returnValue[3];
    UINT16_t DacValue;

    DAC161_Dac_Value = 0;
    if( DAC161_Read_Regs (returnValue, DAC161_DACCODE_REG, 2) )
    {
        DacValue.byte.hi = returnValue[1];
        DacValue.byte.lo = returnValue[2];
        DAC161_Dac_Value = DacValue.val;
        return DAC161_Dac_Value;
    }
    else
        DAC161_Dac_Value = 0xFFFF;

    return DAC161_Dac_Value;
}
/*************************************************************************************************************************************************
*  DAC161_Read_ERR_CONFIG
**************************************************************************************************************************************************/
/*!
* @brief Reads thE ERR_CONFIG register from the DAC161.
*
* This function returns the current value in the DACCODE register of the DAC161
*
* @return  ERR_CONFIG
**************************************************************************************************************************************************/
unsigned short DAC161_Read_ERR_CONFIG (void)
{
    unsigned char returnValue[3];
    UINT16_t DacValue;

    DAC161_Err_Config.Read.Value = 0;
    if( DAC161_Read_Regs (returnValue, DAC161_ERR_CONFIG_REG, 2) )
    {
        DacValue.byte.hi = returnValue[1];
        DacValue.byte.lo = returnValue[2];
        DAC161_Err_Config.Read.Value                    = DacValue.val;
        DAC161_Err_Config.Read.Fields.L_RETRY_TIME      = (unsigned char)((DAC161_Err_Config.Read.Value & DAC161_L_RETRY_TIME_MASK) >> DAC161_L_RETRY_TIME_SHIFT);
        DAC161_Err_Config.Read.Fields.DIS_RETRY_LOOP    = DAC161_Err_Config.Read.Value & DAC161_DIS_RETRY_LOOP_MASK;
        DAC161_Err_Config.Read.Fields.MASK_LOOP_ERR     = DAC161_Err_Config.Read.Value & DAC161_MASK_LOOP_ERR_MASK;
        DAC161_Err_Config.Read.Fields.DIS_LOOP_ERR_ERRB = DAC161_Err_Config.Read.Value & DAC161_LOOP_ERR_ERRB_MASK;
        DAC161_Err_Config.Read.Fields.MASK_SPI_ERR      = DAC161_Err_Config.Read.Value & DAC161_SPI_ERR_MASK;
        DAC161_Err_Config.Read.Fields.SPI_TIMEOUT       = (unsigned char)((DAC161_Err_Config.Read.Value & DAC161_SPI_TIMEOUT_MASK) >> DAC161_SPI_TIMEOUT_SHIFT);
        DAC161_Err_Config.Read.Fields.MASK_SPI_TOUT     = DAC161_Err_Config.Read.Value & DAC161_MASK_SPI_TOUT_MASK;
    }
    else
        DAC161_Err_Config.Read.Value = 0xFFFF;

    return DAC161_Err_Config.Read.Value;
}

unsigned short DAC161_Set_ERR_CONFIG (void)
{

    DAC161_Err_Config.Set.Fields.L_RETRY_TIME          = 0xFF; //DAC161_CONVERT_LOOP_RETRY_TIME( 100 );   // Retry Time 100 ms
    DAC161_Err_Config.Set.Fields.DIS_RETRY_LOOP        = false;                                    // Retry Active
    DAC161_Err_Config.Set.Fields.MASK_LOOP_ERR         = false;                                    // If LOOP error, ouput current is ERR_LOW
    DAC161_Err_Config.Set.Fields.DIS_LOOP_ERR_ERRB     = false;                                    // If LOOP error, pin EERB is set to LOW
    DAC161_Err_Config.Set.Fields.MASK_SPI_ERR          = false;                                    // If SPI error, output current is ERR_OW
    DAC161_Err_Config.Set.Fields.SPI_TIMEOUT           = 0XFF; //DAC161_CONVERT_SPI_TIMEOUT_TIME( 400 );   // Spi Timeout 400 ms
    DAC161_Err_Config.Set.Fields.MASK_SPI_TOUT         = false;                                    // If SPI error, pin EERB is set to LOW


    //IMPOSTAZIONE VALORE DA PASSARE AL REGISTRO*****************************************************************************************************

    DAC161_Err_Config.Set.Value = 0;

    DAC161_Err_Config.Set.Value += (((uint16_t)(DAC161_Err_Config.Set.Fields.L_RETRY_TIME) << DAC161_L_RETRY_TIME_SHIFT) & DAC161_L_RETRY_TIME_MASK);

    if( DAC161_Err_Config.Set.Fields.DIS_RETRY_LOOP == true )
        DAC161_Err_Config.Set.Value |= DAC161_DIS_RETRY_LOOP_MASK;
     else
        DAC161_Err_Config.Set.Value &= ~DAC161_DIS_RETRY_LOOP_MASK;

    if( DAC161_Err_Config.Set.Fields.MASK_LOOP_ERR == true )
        DAC161_Err_Config.Set.Value |= DAC161_MASK_LOOP_ERR_MASK;
    else
        DAC161_Err_Config.Set.Value &= ~DAC161_MASK_LOOP_ERR_MASK;

    if( DAC161_Err_Config.Set.Fields.DIS_LOOP_ERR_ERRB == true )
        DAC161_Err_Config.Set.Value |= DAC161_LOOP_ERR_ERRB_MASK;
    else
        DAC161_Err_Config.Set.Value &= ~DAC161_LOOP_ERR_ERRB_MASK;

    if( DAC161_Err_Config.Set.Fields.MASK_SPI_ERR == true )
        DAC161_Err_Config.Set.Value |= DAC161_SPI_ERR_MASK;
    else
        DAC161_Err_Config.Set.Value &= ~DAC161_SPI_ERR_MASK;

    DAC161_Err_Config.Set.Value += (((uint16_t)(DAC161_Err_Config.Set.Fields.SPI_TIMEOUT) << DAC161_SPI_TIMEOUT_SHIFT) & DAC161_SPI_TIMEOUT_MASK);

    if( DAC161_Err_Config.Set.Fields.MASK_SPI_TOUT == true )
        DAC161_Err_Config.Set.Value |= DAC161_MASK_SPI_TOUT_MASK;
    else
        DAC161_Err_Config.Set.Value &= ~DAC161_MASK_SPI_TOUT_MASK;
    //********************************************************************************************************************************************

    return DAC161_Err_Config.Set.Value;
}
/*************************************************************************************************************************************************
*  DAC161_Read_ERR_LOW
**************************************************************************************************************************************************/
/*!
* @brief Reads thE ERR_LOW register from the DAC161.
*
* This function returns the current value in the DACCODE register of the DAC161
*
* @return  ERR_LOW
**************************************************************************************************************************************************/
unsigned char DAC161_Read_ERR_LOW (void)
{
    unsigned char returnValue[3];

    DAC161_Err_Low_Value = 0;
    if( DAC161_Read_Regs (returnValue, DAC161_ERR_LOW_REG, 2) )
        DAC161_Err_Low_Value = returnValue[1];
    else
        DAC161_Err_Low_Value = 0xFF;

    return DAC161_Err_Low_Value;
}
/*************************************************************************************************************************************************
*  DAC161_Read_ERR_HIGH
**************************************************************************************************************************************************/
/*!
* @brief Reads thE ERR_HIGH register from the DAC161.
*
* This function returns the current value in the DACCODE register of the DAC161
*
* @return  ERR_HIGH
**************************************************************************************************************************************************/
unsigned char DAC161_Read_ERR_HIGH (void)
{
    unsigned char returnValue[3];

    DAC161_Err_High_Value = 0;
    if( DAC161_Read_Regs (returnValue, DAC161_ERR_HIGH_REG, 2) )
        DAC161_Err_High_Value = returnValue[1];
    else
        DAC161_Err_High_Value = 0xFF;

    return DAC161_Err_High_Value;
}

//--

//bool DAC161_SPI_write (unsigned char *outData, unsigned char *inData, unsigned char length)
//{
//    bool WriteOk;

    /* Initialize master SPI transaction structure */
    /*DAC161_Spi_Transaction.count = length;
    DAC161_Spi_Transaction.txBuf = outData;
    DAC161_Spi_Transaction.rxBuf = inData;
    /* Initiate SPI transfer */
    //WriteOk = false;
    //WriteOk = SPI_transfer(DAC161_Spi_Handle, &DAC161_Spi_Transaction);
    //WriteOk = SPI_transfer(DAC161_Spi_Handle, &DAC161_Spi_Transaction);

    //if( WriteOk )
      //return true;
    //else
//      return false;
//}

bool DAC161S_Init(void)
{
    DAC161S_CS_UNSEL;
    return true;
}

//-
//to be called every second
void loopTick(void)
{
    /*if(!MeterParData.str.loopSet.loopEn)
        return;
    if(LoopReInitCnt > 0)
        LoopReInitCnt--;
    else
    {
        LoopReInitCnt = MeterParData.str.loopSet.reInitSeconds;
        doReInit = true;
    }
    doUpdate = true;*/
}

//--

bool loopHandle(void)
{
    /*if(!MeterParData.str.loopSet.loopEn)
        return false;

    uint32_t uA_calc = 0;
    bool ValueOutOfRange = false;

    if(doReInit)
    {
        DAC161_Setup(DAC161_STD_ERR_CONFIG, 3350, 21500);
        Increment420UpdatesCounter();
        doReInit = false;
    }
    if(doUpdate)
    {
        if((MeterFlowrateLPH < 0.0) && (MeterParData.str.loopSet.onlyPositiveFlow))
            uA_calc = 4000;
        else
            uA_calc = (uint32_t)((((fabs(MeterFlowrateLPH) - MeterParData.str.loopSet.flowLPH_4mA) / (MeterParData.str.loopSet.flowLPH_20mA - MeterParData.str.loopSet.flowLPH_4mA)) * 16000.0 ) + 4000.0);
        if(uA_calc < 4000)
        {
            uA_calc = 4000;
            ValueOutOfRange = true;
        }
        else if(uA_calc > 20000)
        {
            uA_calc = 20000;
            ValueOutOfRange = true;
        }
        DAC161_Set_Out_Value_uA(uA_calc);
        Increment420UpdatesCounter();
        doUpdate = false;
    }
    return ValueOutOfRange;*/
}


//--





