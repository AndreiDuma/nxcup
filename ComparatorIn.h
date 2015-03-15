/**************************************************************************************************
 *****                                                                                        *****
 *****  Name: ComparatorIn.h                                                                  *****
 *****  Date: 05/06/2013                                                                      *****
 *****  Auth: Frank Vannieuwkerke                                                             *****
 *****  Func: library for KL25Z Comparator                                                    *****
 *****                                                                                        *****
 **************************************************************************************************/

#ifndef COMPARATORIN_H
#define COMPARATORIN_H
 
/*
 * Includes
 */
#include "mbed.h"
#include "pinmap.h"
 
#ifndef TARGET_KL25Z
    #error "Target not supported"
#endif

/** ComparatorIn library
*
* INP/INM connection selection :
*     PTC6  (IN0)    CMP0_IN0
*     PTC7  (IN1)    CMP0_IN1
*     PTC8  (IN2)    CMP0_IN2
*     PTC9  (IN3)    CMP0_IN3
*     PTE30 (IN4)    CMP0_IN4 (External 12-bit DAC0, auto connect to IN4)
*     PTE29 (IN5)    CMP0_IN5
* Internal 6-bit DAC reference voltage = VDD (3.3V)
* Hysteresis, filter count and sample period are initialised to these values when ComparatorIn is initialized: 
*    CMP0->CR0   = 0x00;  // Filter and digital hysteresis disabled
*    CMP0->CR1   = 0x17;  // Continuous mode, high-speed compare, unfiltered output, output pin disabled
*    CMP0->FPR   = 0x00;  // Filter disabled
*    CMP0->SCR   = 0x06;  // Disable all interrupts and clear flags (flags are cleared by this write)
*    CMP0->DACCR = 0xC4;  // DAC enabled, Vdd is 6bit reference, threshold set to 1/16 of full-scale (0.2V)
*/

typedef enum {
    CMP0_IN0  =  0,
    CMP0_IN1  =  1,
    CMP0_IN2  =  2,
    CMP0_IN3  =  3,
    CMP0_IN4  =  4,
    CMP0_IN5  =  5,
} CMPName;

/** Class to use KL25Z Comparator
*/ 
class ComparatorIn {
 
public:

    /** Create a ComparatorIn, connected to the specified pins.
    * @param pinP = positive ComparatorIn pin to connect to
    * @param pinM = negative ComparatorIn pin to connect to\n
    * @note Valid values for pinP/pinM:\n
    * PTC6, PTC7, PTC8, PTC9, PTE30, PTE29, NC\n
    * Special cases:\n
    * NC    the corresponding input is connected to the internal 6-bit DAC0\n
    * PTE30 PTE30 is set as 12-bit DAC0 output and connected to IN4\n
    * @return none
    */
    ComparatorIn(PinName pinP,PinName pinM);
    
    /** Set the number of consecutive threshold samples.
    * Represents the number of consecutive samples that must agree\n
    * prior to the comparator ouput filter accepting a new output state.\n
    * @param input Unsigned char - range : 1..7
    * @return none
    */
    void FilterCount(unsigned char fico);

    /** Set the hysteresis.
    * 0 : 5mV\n
    * 1 : 10mV\n
    * 2 : 20mV\n
    * 3 : 30mV\n
    * @param input Unsigned char
    * @return none
    */
    void hysteresis(unsigned char hyst);

    /** Sampling mode control.
    * This mode cannot be set when windowing mode is enabled.
    * @param input Unsigned char (0 = disable, 1 = enable)
    * @return none
    */
    void SampleMode(unsigned char samp_en);

    /** Windowing mode control.
    * This mode cannot be set when sampling mode is enabled.
    * @param input Unsigned char (0 = disable, 1 = enable)
    * @return none
    */
    void WindowMode(unsigned char win_en);

    /** Trigger mode control.
    * CMP and DAC are configured to CMP Trigger mode when CMP_CR1[TRIGM] is set to 1.\n
    * In addition, the CMP should be enabled. If the DAC is to be used as a reference\n
    * to the CMP, it should also be enabled.\n
    * CMP Trigger mode depends on an external timer resource to periodically enable the\n
    * CMP and 6-bit DAC in order to generate a triggered compare. Upon setting TRIGM,\n
    * the CMP and DAC are placed in a standby state until an external timer resource\n
    * trigger is received.\n
    * @param input Unsigned char (0 = disable, 1 = enable)
    * @return none
    */
    void TrigMode(unsigned char trig_en);

    /** Power mode control.
    * 0 Low-Speed (LS) Comparison mode selected. In this mode, CMP has\n
    *   slower output propagation delay and lower current consumption.\n
    * 1 High-Speed (HS) Comparison mode selected. In this mode, CMP has\n
    *   faster output propagation delay and higher current consumption.\n
    * @param input Unsigned char - (0 = Low-Speed, 1 = high-Speed)
    * @return none
    */
    void PowerMode(unsigned char pmode);

    /** Invert mode control.
    * Allows selection of the polarity of the analog comparator function.\n
    * It is also driven to the COUT output, on both the device pin and as SCR[COUT], when OPE=0.\n
    * 0 Does not invert the comparator output.\n
    * 1 Inverts the comparator output.\n
    * @param input Unsigned char - (0 = not inverted, 1 = inverted)
    * @return none
    */
    void invert(unsigned char inv);

    /** Comparator Output Select.
    * 0 Set the filtered comparator output (CMPO) to equal COUT.\n
    * 1 Set the unfiltered comparator output (CMPO) to equal COUTA.\n
    * @param input Unsigned char - (0 : CMPO = COUT, 1 : CMPO = COUTA)
    * @return none
    */
    void OutputSelect(unsigned char cos);

    /** Connect the comparator Output Pin to an external pin.
    * Only one pin can be connected at any time.\n
    * Each time this function is called, the last active pin will be disabled before the new pin is enabled.\n
    * @param input NC   disconnect CMPO from the associated CMPO output pin.
    * @param input PTC0 connect CMPO to PTC0.
    * @param input PTC5 connect CMPO to PTC5.
    * @param input PTE0 connect CMPO to PTE0.
    */
    void OutputPin(PinName ope);

    /** Comparator Module control.
    * Used to switch the Analog Comparator module on/off. When the module is not enabled,\n
    * it remains in the off state, and consumes no power. When the user selects the same\n
    * input from analog mux to the positive and negative port, the comparator is disabled automatically.\n
    * 0 Analog Comparator is disabled.\n
    * 1 Analog Comparator is enabled.\n
    * @param input Unsigned char - (0 = disable, 1 = enable)
    * @return none
    */
    void enable(unsigned char en);

    /** Set the filter sample period.
    * Specifies the sampling period, in bus clock cycles, of the comparator\n
    * output filter, when CR1[SE]=0. Setting FILT_PER to 0x0 disables the filter.\n
    * @param input Unsigned char - range : 0..255
    * @return none
    */
    void FilterPeriod(unsigned char fipe);

    /** DMA Control.
    * Used to switch the DMA transfer triggered from the CMP module on/off.\n
    * When this field is set, a DMA request is asserted when CFR or CFF is set.\n
    * 0 DMA is disabled.\n
    * 1 DMA is enabled.\n
    * @param input Unsigned char - (0 = disable, 1 = enable)
    * @return none
    */
    void dma(unsigned char dmaen);

    /** Analog Comparator Output.
    * Returns the current value of the Analog Comparator output.\n
    * The field is reset to 0 and will read as CR1[INV] when the Analog Comparator\n
    * module is disabled, that is, when CR1[EN] = 0. Writes to this field are ignored.\n
    * @param none
    * @return comparator status (unsigned char)
    */
    unsigned char status(void);

    /** DAC Control.
    * Used to switch the internal DAC on/off. When the DAC is disabled, it is powered down to conserve power.\n
    * 0 DAC is disabled.\n
    * 1 DAC is enabled.\n
    * @param input Unsigned char - 0 or 1
    * @return none
    */
    void dac(unsigned char den);

    /** Supply Voltage Reference Source Select.
    * 0 - V is selected as resistor ladder network supply reference Vin1 = VREFH\n
    * 1 - V is selected as resistor ladder network supply reference Vin2 = VDD\n
    *     (Use this option for the best ADC operation).\n
    * @param input Unsigned char - 0 or 1
    * @return none
    */
    void RefSource(unsigned char res);

    /** Set the detection threshold level (DAC Output Voltage Select).
    * Sets The 6-bit or 12-bit DAC output voltage, depending on which DAC is selected on init.\n
    *  6-bit DACO range is from Vin/64 to Vin.\n
    * 12-bit DACO range is from Vin/4096 to Vin.\n
    * @param input float - range 0.0 .. 1.0
    * @return none
    */
    void treshold(float vo_pct);

    /** Pass Through Mode Control.
    * Set the MUX pass through mode. Pass through mode is always available, but for\n
    * some devices, this feature must be always disabled due to the lack of package pins.\n
    * 0 Pass Through Mode is disabled.\n
    * 1 Pass Through Mode is enabled.\n
    * @param input Unsigned char - (0 = disable, 1 = enable)
    * @return none
    */
    void PassThrough(unsigned char ptm);

    /** Plus Input Mux Control.
    * Determines which input is selected for the plus input of the comparator.\n
    * For INx inputs, see CMP, DAC, and ANMUX block diagrams.\n
    * 0 : IN0   PTC6    CMP0_IN0\n
    * 1 : IN1   PTC7    CMP0_IN1\n
    * 2 : IN2   PTC8    CMP0_IN2\n
    * 3 : IN3   PTC9    CMP0_IN3\n
    * 4 : IN4   PTE30   CMP0_IN4 12-bit DAC0\n
    * 5 : IN5   PTE29   CMP0_IN5\n
    * 6 : IN6   -       Bandgap reference (1V)\n
    * 7 : IN7   -       Internal 6-bit DAC0\n
    * @note When an inappropriate operation selects the same input for both muxes, the comparator\n
    * automatically shuts down to prevent itself from becoming a noise generator.\n
    * @note When using the PMC bandgap 1V reference voltage as CMP input, ensure that\n
    * @you enable the bandgap buffer by setting the PMC_REGSC[BGBE] bit.\n
    * @param input Unsigned char - range 0..7
    * @return none
    */
    void SwitchPlus(unsigned char pinP);

    /** Minus Input Mux Control.
    * Determines which input is selected for the plus input of the comparator.\n
    * For INx inputs, see CMP, DAC, and ANMUX block diagrams.\n
    * 0 : IN0   PTC6    CMP0_IN0\n
    * 1 : IN1   PTC7    CMP0_IN1\n
    * 2 : IN2   PTC8    CMP0_IN2\n
    * 3 : IN3   PTC9    CMP0_IN3\n
    * 4 : IN4   PTE30   CMP0_IN4 12-bit DAC0\n
    * 5 : IN5   PTE29   CMP0_IN5\n
    * 6 : IN6   -       Bandgap reference (1V)\n
    * 7 : IN7   -       Internal 6-bit DAC0\n
    * @note When an inappropriate operation selects the same input for both muxes, the comparator\n
    * automatically shuts down to prevent itself from becoming a noise generator.\n
    * @note When using the PMC bandgap 1V reference voltage as CMP input, ensure that
    * you enable the bandgap buffer by setting the PMC_REGSC[BGBE] bit.
    * @param input Unsigned char - range 0..7
    * @return none
    */
    void SwitchMin(unsigned char pinM);

    /** Comparator rising interrupt callback.
    * @param pointer to the user function to execute after IRQ assertion
    * @param NULL to disable the interrupt
    * @return none
    * @note The interrupt is automatically enabled when a valid pointer is used.\n
    * The interrupt is automatically disabled when both risng and falling are set to NULL.
    */
    void rising(void(*fptr)(void));

    /** Comparator falling interrupt callback
    * @param pointer to the user function to execute after IRQ assertion
    * @param NULL to disable the interrupt
    * @return none
    * @note The interrupt is automatically enabled when a valid pointer is used.\n
    * The interrupt is automatically disabled when both risng and falling are set to NULL.
    */
    void falling(void(*fptr)(void));

private:
    static const PinMap PinMap_CMP[7];
    static void _cmpISR(void);
    void hscmp_clear(void);
    PinName op_status(void);
    void op_enable(PinName pen, PinName pstat);
    void op_disable(PinName pdi);
    void dac6_write(unsigned int value);
    char CMPnumberP, CMPnumberM;
};
 
#endif



