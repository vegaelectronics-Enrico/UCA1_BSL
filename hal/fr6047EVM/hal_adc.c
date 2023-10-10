
//#############################################################################
//
//! \file   hal_adc.c
//!
//! \brief  Hardware Abstraction Layer for ADC functions
//!
//!  Group:          MSP Smart Metering Systems Applications
//!  Target Device:  MSP430FR6047
//!
//!  (C) Copyright 2019, Texas Instruments, Inc.
//#############################################################################

#include <stdint.h>
#include <stdbool.h>
#include "hal.h"
#include "System/SysStatus.h"

#ifdef ENABLE_ADC

//
// Variables
//
/*! Flag indicating if temperature sensor measurement is ready */
static bool temp_sensor_rdy;
/*! Flag indicating if voltage supply measurement is ready */
static bool volt_supply_rdy;
/*! Flag indicating if current supply measurement is ready */
static bool curr_supply_rdy;
/*! ADC result of temperature sensor */
static uint16_t temp_sensor_raw;
/*! ADC result of voltage supply */
static uint16_t volt_supply_raw;
/*! ADC result of current supply */
static uint16_t curr_supply_raw;
/*! Pointer to REF calibration constants in TLV structure */
static const uint16_t *cal_ref_ptr;
/*! Pointer to ADC calibration constants in TLV structure */
static const uint16_t *cal_adc_ptr;
/*! Constant used for calculation of temperature conversion */
static float temp_const;

/*! TLV offset for 2.0V REF calibration */
#define CAL_ADC_20VREF_FACTOR   (1)
/*! TLV offset for ADC Gain calibration factor */
#define CAL_ADC_GAIN_FACTOR     (0)
/*! TLV offset for ADC offset */
#define CAL_ADC_OFFSET          (1)
/*! TLV offset for temperature sensor calibration at 30C using 2.0V REF */
#define CAL_ADC_20T30           (4)
/*! TLV offset for temperature sensor calibration at 85C using 2.0V REF */
#define CAL_ADC_20T85           (5)




void hal_adc_init(void)
{
    uint16_t *ptr_tlv;
    uint8_t len;

    temp_sensor_rdy = false;

    TLV_getInfo(TLV_TAG_ADC12CAL, 0, &len, &ptr_tlv);
    cal_adc_ptr = (const uint16_t *) ptr_tlv;

    TLV_getInfo(TLV_TAG_REFCAL, 0, &len, &ptr_tlv);
    cal_ref_ptr = (const uint16_t *) ptr_tlv;

    // calculate constant to be used during temperature conversions
    temp_const = (float)(85-30)/(float)(cal_adc_ptr[CAL_ADC_20T85] -
                                        cal_adc_ptr[CAL_ADC_20T30]);

    // Configure GPIOs used for ADC measurements
    // P1.1 used as VCC_SENSE_EN. Should be enabled as output low to enable voltage divider
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN1,
                                               GPIO_TERNARY_MODULE_FUNCTION);
    //P1.0 used for supply current read (ENRICO)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN0,
                                               GPIO_TERNARY_MODULE_FUNCTION);

    //If ref generator busy, WAIT
    while(Ref_A_isRefGenBusy(REF_A_BASE))
    {
        ;
    }

    //Select internal ref = 2.0V
    Ref_A_setReferenceVoltage(REF_A_BASE,
                              REF_A_VREF2_0V);


    //Initialize the ADC12B Module
    /*
     * Base address of ADC12B Module
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider/pre-divider of 1
     * Not use internal channel
     */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    //initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ACLK; //Enrico 0.20
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_TEMPSENSEMAP;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    // Disable ADC12 module by default
    //ADC12_B_enable(ADC12_B_BASE);

    /*
     * Base address of ADC12B Module
     * For memory buffers 0-7 sample/hold for 256 clock cycles (47us-64us using MODOSC)
     * For memory buffers 8-15 sample/hold for 32 clock cycles (5.9us - 8us using MODOSC)
     * Disable Multiple Sampling
     */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_256_CYCLES,
                               ADC12_B_CYCLEHOLD_256_CYCLES,
                               ADC12_B_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A30 (temp sensor) to memory buffer 0
     * Vref+ = IntBuffer
     * Vref- = AVss
     */
    ADC12_B_configureMemoryParam configureMemoryParam = {0};
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_TCMAP;
    configureMemoryParam.refVoltageSourceSelect =
                                            ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect =
                                            ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect =
                                            ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 8
     * input A1 (external voltage divider) to memory buffer 8
     * Vref+ = IntBuffer
     * Vref- = AVss
     */
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_8;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A1;
    configureMemoryParam.refVoltageSourceSelect =
                                            ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect =
                                            ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect =
                                            ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 16
     * input A0 (battery current integrator) to memory buffer 16
     * Vref+ = IntBuffer
     * Vref- = AVss
     */
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A0;
    configureMemoryParam.refVoltageSourceSelect =
                                            ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect =
                                            ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect =
                                            ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

}

float hal_adc_tempsensor_readCelsius(void)
{
    int32_t adc_corrected;
    float temperature_Celsius;


    //Enable ADC12 module; takes 100ns to stabilize
    ADC12_B_enable(ADC12_B_BASE);

    //Turn on Reference Voltage
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    while (Ref_A_isVariableReferenceVoltageOutputReady(REF_A_BASE) != REF_A_READY)
        ;

    __disable_interrupt();
    temp_sensor_rdy = false;

    ADC12_B_clearInterrupt(ADC12_B_BASE,
                           0,
                           ADC12_B_IFG0
                           );

    //Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,
                            ADC12_B_IE0,
                            0,
                            0);
    /*
     * Base address of ADC12B Module
     * Start the conversion into memory buffer 0
     * Use the single-channel, single-conversion mode
     */
    ADC12_B_startConversion(ADC12_B_BASE,
                            ADC12_B_START_AT_ADC12MEM0,
                            ADC12_B_SINGLECHANNEL);

    while (temp_sensor_rdy == false)
    {
        __bis_SR_register(LPM0_bits + GIE);    // LPM0, ADC10_ISR will force exit
        __no_operation();                      // For debug only
        __disable_interrupt();
    }
    __enable_interrupt();
    ADC12_B_disableInterrupt(ADC12_B_BASE,
                                ADC12_B_IE0,
                                0,
                                0);

    // Adjust for 2.0V reference error
    // ADC(corrected) = ADC(raw) x CAL_ADC20VREF_FACTOR x 1/2^15
    adc_corrected = (int32_t) temp_sensor_raw * cal_ref_ptr[CAL_ADC_20VREF_FACTOR];
    adc_corrected >>= 15;

    // Adjust for Gain
    //ADC(gain_corrected) = ADC(raw) x CAL_ADC_GAIN_FACTOR x 1/2^15
    adc_corrected = (int32_t) temp_sensor_raw * cal_adc_ptr[CAL_ADC_GAIN_FACTOR];
    adc_corrected >>= 15;

    // Adjust for offset
    // ADC(offset_corrected ) = ADC(raw) +CAL_ADC_OFFSET
    adc_corrected = temp_sensor_raw + (int16_t)cal_adc_ptr[CAL_ADC_OFFSET];

    // Calculate temperature
    // Temp[C] = [ADC(raw) - CAL_ADC_20T30] x [(85-30)/(CAL_ADC_20T85 - CAL_ADC_20T30)] + 30
    adc_corrected = (int32_t) adc_corrected - cal_adc_ptr[CAL_ADC_20T30];
    temperature_Celsius = adc_corrected * temp_const + 30;

    //Turn off Reference Voltage
    Ref_A_disableReferenceVoltage(REF_A_BASE);

    // Disable ADC12 module
    ADC12_B_disable(ADC12_B_BASE);


    return temperature_Celsius;

}

uint16_t hal_adc_voltagesupply_readmV(void)
{
    int32_t adc_corrected;
    uint16_t adc_mv;

    // turn on VCC_SENSE_EN
    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
    //GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);

    //Enable ADC12 module; takes 100ns to stabilize
    ADC12_B_enable(ADC12_B_BASE);

    //Turn on Reference Voltage
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    while (Ref_A_isVariableReferenceVoltageOutputReady(REF_A_BASE) != REF_A_READY)
        ;

    __disable_interrupt();
    volt_supply_rdy = false;

    ADC12_B_clearInterrupt(ADC12_B_BASE,
                           0,
                           ADC12_B_IFG8
                           );

    //Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,
                            ADC12_B_IE8,
                            0,
                            0);
    /*
     * Base address of ADC12B Module
     * Start the conversion into memory buffer 8
     * Use the single-channel, single-conversion mode
     */
    ADC12_B_startConversion(ADC12_B_BASE,
                            ADC12_B_START_AT_ADC12MEM8,
                            ADC12_B_SINGLECHANNEL);

    while (volt_supply_rdy == false)
    {
        __bis_SR_register(LPM0_bits + GIE);    // LPM0, ADC10_ISR will force exit
        __no_operation();                      // For debug only
        __disable_interrupt();
    }
    __enable_interrupt();
    ADC12_B_disableInterrupt(ADC12_B_BASE,
                                ADC12_B_IE8,
                                0,
                                0);

    // DIsable VCC_SENSE_EN
    //GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN2);

    // Adjust for 2.0V reference error
    // ADC(corrected) = ADC(raw) x CAL_ADC20VREF_FACTOR x 1/2^15
    adc_corrected = (int32_t) volt_supply_raw * cal_ref_ptr[CAL_ADC_20VREF_FACTOR];
    adc_corrected >>= 15;

    // Adjust for Gain
    //ADC(gain_corrected) = ADC(raw) x CAL_ADC_GAIN_FACTOR x 1/2^15
    adc_corrected = (int32_t) adc_corrected * cal_adc_ptr[CAL_ADC_GAIN_FACTOR];
    adc_corrected >>= 15;

    // Adjust for offset
    // ADC(offset_corrected ) = ADC(raw) +CAL_ADC_OFFSET
    adc_corrected = adc_corrected + (int16_t)cal_adc_ptr[CAL_ADC_OFFSET];

    // Maximum count (4096) is equal to 2V,
    // but since the Vin is divided by 2, then the maximum is 4V (4000mV)
    //Enrico > same in our board
    adc_mv = (uint32_t) 4000 *  (uint32_t) adc_corrected / (uint32_t) 4096;

    //Turn off Reference Voltage
    Ref_A_disableReferenceVoltage(REF_A_BASE);

    // Disable ADC12 module
    ADC12_B_disable(ADC12_B_BASE);

    return (uint16_t) adc_mv;
}

//--

uint16_t hal_adc_currentsupply_readuA(void)
{
    int32_t adc_corrected;
    uint16_t adc_mv;

    //Enable ADC12 module; takes 100ns to stabilize
    ADC12_B_enable(ADC12_B_BASE);

    //Turn on Reference Voltage
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    while (Ref_A_isVariableReferenceVoltageOutputReady(REF_A_BASE) != REF_A_READY)
        ;

    __disable_interrupt();
    curr_supply_rdy = false;

    ADC12_B_clearInterrupt(ADC12_B_BASE,
                           0,
                           ADC12_B_IFG1
                           );

    //Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,
                            ADC12_B_IE1,
                            0,
                            0);
    /*
     * Base address of ADC12B Module
     * Start the conversion into memory buffer 16
     * Use the single-channel, single-conversion mode
     */
    ADC12_B_startConversion(ADC12_B_BASE,
                            ADC12_B_START_AT_ADC12MEM1,
                            ADC12_B_SINGLECHANNEL);

    while (curr_supply_rdy == false)
    {
        __bis_SR_register(LPM0_bits + GIE);    // LPM0, ADC10_ISR will force exit
        __no_operation();                      // For debug only
        __disable_interrupt();
    }
    __enable_interrupt();
    ADC12_B_disableInterrupt(ADC12_B_BASE,
                                ADC12_B_IE1,
                                0,
                                0);

    // DIsable VCC_SENSE_EN
    //GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN2);

    // Adjust for 2.0V reference error
    // ADC(corrected) = ADC(raw) x CAL_ADC20VREF_FACTOR x 1/2^15
    adc_corrected = (int32_t) curr_supply_raw * cal_ref_ptr[CAL_ADC_20VREF_FACTOR];
    adc_corrected >>= 15;

    // Adjust for Gain
    //ADC(gain_corrected) = ADC(raw) x CAL_ADC_GAIN_FACTOR x 1/2^15
    adc_corrected = (int32_t) adc_corrected * cal_adc_ptr[CAL_ADC_GAIN_FACTOR];
    adc_corrected >>= 15;

    // Adjust for offset
    // ADC(offset_corrected ) = ADC(raw) +CAL_ADC_OFFSET
    adc_corrected = adc_corrected + (int16_t)cal_adc_ptr[CAL_ADC_OFFSET];

    // Maximum count (4096) is equal to 2V,
    adc_mv = (uint32_t) 2000 *  (uint32_t) adc_corrected / (uint32_t) 4096;

    //Turn off Reference Voltage
    Ref_A_disableReferenceVoltage(REF_A_BASE);

    // Disable ADC12 module
    ADC12_B_disable(ADC12_B_BASE);

    //il gain dell'operazionale su shunt da 1 ohm è 1000, quindi i mV corrispondono ai uA
    return (uint16_t) adc_mv + CURR_MEAS_CIRCUIT_uA;
}



//*****************************************************************************
//
//! Interrupt Service routine for ADC12
//!
//! Sets a flag to indicate that an ADC result is complete and returns value
//!
//! \return none
//
// *****************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
#endif
void ADC12_ISR(void)
{
    switch(__even_in_range(ADC12IV,76))
    {
    case  0: break;                         // Vector  0:  No interrupt
    case  2: break;                         // Vector  2:  ADC12BMEMx Overflow
    case  4: break;                         // Vector  4:  Conversion time overflow
    case  6: break;                         // Vector  6:  ADC12BHI
    case  8: break;                         // Vector  8:  ADC12BLO
    case 10: break;                         // Vector 10:  ADC12BIN
    case 12:                                // Vector 12:  ADC12BMEM0 Interrupt
        temp_sensor_rdy = true;
        temp_sensor_raw = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0);
        __bic_SR_register_on_exit(LPM3_bits);           // Exit active CPU
        break;                              // Clear CPUOFF bit from 0(SR)
    case 14:                          // Vector 14:  ADC12BMEM1
        curr_supply_rdy = true;
        curr_supply_raw = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1);
        __bic_SR_register_on_exit(LPM3_bits);
    break;
    case 16: break;                         // Vector 16:  ADC12BMEM2
    case 18: break;                         // Vector 18:  ADC12BMEM3
    case 20: break;                         // Vector 20:  ADC12BMEM4
    case 22: break;                         // Vector 22:  ADC12BMEM5
    case 24: break;                         // Vector 24:  ADC12BMEM6
    case 26: break;                         // Vector 26:  ADC12BMEM7
    case 28:                                // Vector 28:  ADC12BMEM8
        volt_supply_rdy = true;
        volt_supply_raw = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_8);
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 30: break;                         // Vector 30:  ADC12BMEM9
    case 32: break;                         // Vector 32:  ADC12BMEM10
    case 34: break;                         // Vector 34:  ADC12BMEM11
    case 36: break;                         // Vector 36:  ADC12BMEM12
    case 38: break;                         // Vector 38:  ADC12BMEM13
    case 40: break;                         // Vector 40:  ADC12BMEM14
    case 42: break;                         // Vector 42:  ADC12BMEM15
    case 44: break;                         // Vector 44:  ADC12BMEM16
    case 46: break;                         // Vector 46:  ADC12BMEM17
    case 48: break;                         // Vector 48:  ADC12BMEM18
    case 50: break;                         // Vector 50:  ADC12BMEM19
    case 52: break;                         // Vector 52:  ADC12BMEM20
    case 54: break;                         // Vector 54:  ADC12BMEM21
    case 56: break;                         // Vector 56:  ADC12BMEM22
    case 58: break;                         // Vector 58:  ADC12BMEM23
    case 60: break;                         // Vector 60:  ADC12BMEM24
    case 62: break;                         // Vector 62:  ADC12BMEM25
    case 64: break;                         // Vector 64:  ADC12BMEM26
    case 66: break;                         // Vector 66:  ADC12BMEM27
    case 68: break;                         // Vector 68:  ADC12BMEM28
    case 70: break;                         // Vector 70:  ADC12BMEM29
    case 72: break;                         // Vector 72:  ADC12BMEM30
    case 74: break;                         // Vector 74:  ADC12BMEM31
    case 76: break;                         // Vector 76:  ADC12BRDY
    default: break;
    }
}

#endif
