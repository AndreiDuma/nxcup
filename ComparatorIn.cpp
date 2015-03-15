/**************************************************************************************************
 *****                                                                                        *****
 *****  Name: ComparatorIn.cpp                                                                *****
 *****  Date: 05/06/2013                                                                      *****
 *****  Auth: Frank Vannieuwkerke                                                             *****
 *****  Func: library for KL25Z Comparator                                                    *****
 *****                                                                                        *****
 **************************************************************************************************/

#include "ComparatorIn.h"
 
void (*comparatorin_rise_fptr)(void);     // Pointer to user function - called after rising IRQ assertion.
void (*comparatorin_fall_fptr)(void);     // Pointer to user function - called after falling IRQ assertion.
AnalogOut *_dac12;                        // Pointer to AnalogOut

const PinMap ComparatorIn::PinMap_CMP[] = {
    {PTC6,  CMP0_IN0, 0},
    {PTC7,  CMP0_IN1, 0},
    {PTC8,  CMP0_IN2, 0},
    {PTC9,  CMP0_IN3, 0},
    {PTE30, CMP0_IN4, 0},       // 12-bit DAC0
    {PTE29, CMP0_IN5, 0},       // ADC0_SE4b
    {NC,    NC,       0}        // Internal 6-bit DAC0
};


ComparatorIn::ComparatorIn(PinName pinP, PinName pinM)
{
    comparatorin_rise_fptr = NULL;
    comparatorin_fall_fptr = NULL;
    CMPnumberP = (CMPName)pinmap_peripheral(pinP, PinMap_CMP);
    if (CMPnumberP == (uint32_t)NC)         // When NC, use DAC0
        CMPnumberP = 0x07;
    CMPnumberM = (CMPName)pinmap_peripheral(pinM, PinMap_CMP);
    if (CMPnumberM == (uint32_t)NC)         // When NC, use DAC0
        CMPnumberM = 0x07;

    SIM->SCGC4 |=SIM_SCGC4_CMP_MASK; // Enable HSCMP module clock

    hscmp_clear();
    CMP0->CR0   = 0x00;  // Filter and digital hysteresis disabled
    CMP0->CR1   = 0x17;  // Continuous mode, high-speed compare, unfiltered output, output pin disabled
    CMP0->FPR   = 0x00;  // Filter disabled
    CMP0->SCR   = 0x06;  // Disable all interrupts and clear flags (flags are cleared by this write)
    CMP0->DACCR = 0xE0;  // DAC enabled, Vdd is 6bit reference, threshold set to 1/2 of full-scale (1.65V)
    CMP0->MUXCR = (CMPnumberP << 3) | (CMPnumberM & 0x07);  // P-input/M-input are ext.channels defined by CMPnumberP/CMPnumberN

    if(CMPnumberP < 6) pinmap_pinout(pinP, PinMap_CMP); // Map pins
    if(CMPnumberM < 6) pinmap_pinout(pinM, PinMap_CMP); // Map pins

    if((CMPnumberP == 4) || (CMPnumberM == 4)) _dac12 = new AnalogOut (PTE30);  // When PTE30 is selected, use it as 12-bit DAC
    
    NVIC_SetVector(CMP0_IRQn, (uint32_t)&_cmpISR);      // Set comparator ISR to _cmpISR routine
    falling(NULL);                                      // set falling IRQ pointer to NULL
    rising(NULL);                                       // set rising IRQ pointer to NULL
    NVIC_DisableIRQ(CMP0_IRQn);                         // disable CMP0 IRQ
};
 
void ComparatorIn::FilterCount(unsigned char fico)
{
    if((fico > 0) && (fico < 8))
    {
        unsigned char tmp;
        tmp = (CMP0->CR0 & 0x8F) | CMP_CR0_FILTER_CNT(fico);        // Replace old value
        CMP0->CR0 = tmp;                                            // Set filter count
    }
}

void ComparatorIn::hysteresis(unsigned char hyst)
{
    if(hyst < 4)
    {
        unsigned char tmp;
        tmp = (CMP0->CR0 & 0xFC) | CMP_CR0_HYSTCTR(hyst);           // Replace old value
        CMP0->CR0 = tmp;                                            // Set hysteresis
    }
}

void ComparatorIn::SampleMode(unsigned char samp_en)
{
    if((CMP0->CR1 & CMP_CR1_WE_MASK) == 0)                  // Only allow change when window mode is inactive
    {
        if(samp_en == 1) CMP0->CR1 |= CMP_CR1_SE_MASK;      // Enable
        else CMP0->CR1 &= ~CMP_CR1_SE_MASK;                 // Disable
    }
}

void ComparatorIn::WindowMode(unsigned char win_en)
{
    if((CMP0->CR1 & CMP_CR1_SE_MASK) == 0)                  // Only allow change when sample mode is inactive
    {
        if(win_en == 1) CMP0->CR1 |= CMP_CR1_WE_MASK;       // Enable
        else CMP0->CR1 &= ~CMP_CR1_WE_MASK;                 // Disable
    }
}

void ComparatorIn::TrigMode(unsigned char trig_en)
{
    if(trig_en == 1) CMP0->CR1 |= CMP_CR1_TRIGM_MASK;       // Enable
    else CMP0->CR1 &= ~CMP_CR1_TRIGM_MASK;                  // Disable
}

void ComparatorIn::PowerMode(unsigned char pmode)
{
    if(pmode == 1) CMP0->CR1 |= CMP_CR1_PMODE_MASK;      // Set high speed
    else CMP0->CR1 &= ~CMP_CR1_PMODE_MASK;               // Set low speed
}

void ComparatorIn::invert(unsigned char inv)
{
    if(inv == 1) CMP0->CR1 |= CMP_CR1_INV_MASK;          // Enable
    else CMP0->CR1 &= ~CMP_CR1_INV_MASK;                 // Disable
}

void ComparatorIn::OutputSelect(unsigned char cos)
{
    if(cos == 1) CMP0->CR1 |= CMP_CR1_COS_MASK;          // Enable
    else CMP0->CR1 &= ~CMP_CR1_COS_MASK;                 // Disable
}

void ComparatorIn::OutputPin(PinName ope)
{
    PinName pin_stat;
    pin_stat = op_status();                                  // Get pin status
    // Only change settings if new pin differs from old pin AND the correct pin is selected.
    if((ope != pin_stat) && ((ope == PTC0) || (ope == PTC5) || (ope == PTE0) || (ope == NC)))
    {
        if(ope == NC)
        {
            if (pin_stat != NC) op_disable(pin_stat);        // disconnect current pin
            CMP0->CR1 &= ~CMP_CR1_OPE_MASK;                  // Disable comparator output pin connect
        }
        else
        {
            op_enable(ope, pin_stat);                        // Connect new pin
            CMP0->CR1 &= ~CMP_CR1_OPE_MASK;                  // Enable comparator output pin connect
        }
    }
}

void ComparatorIn::enable(unsigned char en)
{
    if(en == 1) CMP0->CR1 |= CMP_CR1_EN_MASK;            // Enable
    else CMP0->CR1 &= ~CMP_CR1_EN_MASK;                  // Disable
}

void ComparatorIn::FilterPeriod(unsigned char fipe)
{
    CMP0->FPR = CMP_FPR_FILT_PER(fipe);
}

void ComparatorIn::dma(unsigned char dmaen)
{
    if(dmaen == 1) CMP0->SCR |= CMP_SCR_DMAEN_MASK;      // Enable
    else CMP0->SCR &= ~CMP_SCR_DMAEN_MASK;               // Disable
}

unsigned char ComparatorIn::status(void)
{
    return (CMP0->SCR & 0x01);
}

void ComparatorIn::dac(unsigned char den)
{
    if(den == 1) CMP0->DACCR |= CMP_DACCR_DACEN_MASK;    // Enable
    else CMP0->DACCR &= ~CMP_DACCR_DACEN_MASK;           // Disable
}

void ComparatorIn::RefSource(unsigned char res)
{
    if(res == 1) CMP0->DACCR |= CMP_DACCR_VRSEL_MASK;    // Enable
    else CMP0->DACCR &= ~CMP_DACCR_VRSEL_MASK;           // Disable
}

void ComparatorIn::treshold(float vo_pct)
{
    
    if(vo_pct < 0.0) vo_pct = 0.0;
    if(vo_pct > 1.0) vo_pct = 1.0;;

    if((CMPnumberP == 7) || (CMPnumberM == 7))
    {
        dac6_write(vo_pct * (float)0x3F);
    }
    if((CMPnumberP == 4) || (CMPnumberM == 4))
    {
        _dac12->write(vo_pct);
    }
}

void ComparatorIn::PassThrough(unsigned char ptm)
{
    if(ptm == 1) CMP0->MUXCR |= CMP_MUXCR_MSEL_MASK;     // Enable
    else CMP0->MUXCR &= ~CMP_MUXCR_MSEL_MASK;            // Disable
}

void ComparatorIn::SwitchPlus(unsigned char pinP)
{
}
 
void ComparatorIn::SwitchMin(unsigned char pinM)
{
}
 
void ComparatorIn::hscmp_clear(void)
{
  CMP0->CR0   = 0;
  CMP0->CR1   = 0;
  CMP0->FPR   = 0;
  CMP0->SCR   = 0x06;  // Clear flags if set.
  CMP0->DACCR = 0;
  CMP0->MUXCR = 0;
}

void ComparatorIn::rising(void(*fptr)(void))
{
    if(fptr == NULL)
    {
        CMP0->SCR &= ~CMP_SCR_IER_MASK;  // Disable rising int.
        CMP0->SCR |=  CMP_SCR_CFR_MASK;  // clear flag
        if(comparatorin_fall_fptr == NULL)
            NVIC_DisableIRQ(CMP0_IRQn);
    }
    else
    {
        comparatorin_rise_fptr = fptr;
        CMP0->SCR |= (CMP_SCR_IER_MASK | CMP_SCR_CFR_MASK);  // Enable rising int. and clear flag
        NVIC_EnableIRQ(CMP0_IRQn);  // enable CMP0 interrupt
    }
}
 
void ComparatorIn::falling(void(*fptr)(void))
{
    if(fptr == NULL)
    {
        CMP0->SCR &= ~CMP_SCR_IEF_MASK;  // Disable falling int.
        CMP0->SCR |=  CMP_SCR_CFF_MASK;  // clear flag
        if(comparatorin_rise_fptr == NULL)
            NVIC_DisableIRQ(CMP0_IRQn);
    }
    else
    {
        comparatorin_fall_fptr = fptr;
        CMP0->SCR |= (CMP_SCR_IEF_MASK | CMP_SCR_CFF_MASK);  // Enable falling int. and clear flag
        NVIC_EnableIRQ(CMP0_IRQn);  // enable CMP0 interrupt
    }
}

void ComparatorIn::_cmpISR(void)  
{
    // Interrupt flags are cleared by writing 1 to the CFx flag
    // Rising edge
    if (((CMP0->SCR & CMP_SCR_IER_MASK)==CMP_SCR_IER_MASK) && ((CMP0->SCR & CMP_SCR_CFR_MASK)==CMP_SCR_CFR_MASK))
    {
        CMP0->SCR |= CMP_SCR_CFR_MASK;                                // Clear the flag
        if (comparatorin_rise_fptr != NULL) comparatorin_rise_fptr(); // call user function
    }

    // Falling edge
    if (((CMP0->SCR & CMP_SCR_IEF_MASK)==CMP_SCR_IEF_MASK) && ((CMP0->SCR & CMP_SCR_CFF_MASK)==CMP_SCR_CFF_MASK))
    {
        CMP0->SCR |= CMP_SCR_CFF_MASK;                                // Clear the flag
        if (comparatorin_fall_fptr != NULL) comparatorin_fall_fptr(); // call user function
    } 
}

/*
IMPORTANT : Do not alter the if... sequence in op_status.
            We need to check if the port is active (using SIM->SCGC5) BEFORE reading PORTn->PCR[x].
            Reading PORTn->PCR[x] while a port is inactive will block the system.
            At startup, SIM->SCGC5 = 00000380h. This means only PORTA is enabled.
*/
PinName ComparatorIn::op_status(void)
{
    if((SIM->SCGC5 & SIM_SCGC5_PORTE_MASK) == 1)
    {
        if((PORTE->PCR[0] & PORT_PCR_MUX_MASK) == 0x500u) return(PTE0);  // check if current pin = PTE0
    }
    if((SIM->SCGC5 & SIM_SCGC5_PORTC_MASK) == 1)
    {
        if((PORTC->PCR[0] & PORT_PCR_MUX_MASK) == 0x500u) return(PTC0);  // check if current pin = PTC0
        if((PORTC->PCR[5] & PORT_PCR_MUX_MASK) == 0x600u) return(PTC5);  // check if current pin = PTC5
    }
    return(NC);
}

void ComparatorIn::op_enable(PinName pen, PinName pstat)
{
    if(pstat != NC) op_disable(pstat);               // If a pin is connected - disconnect before connecting new pin
    switch (pen)
    {
        case PTC0:
            if((SIM->SCGC5 & SIM_SCGC5_PORTC_MASK) == 0) SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;    // If PORTC is inactive: Enable
            PORTC->PCR[0] = PORT_PCR_MUX(5);                                                    // Set PTC0 mux to CMP0
            break;
        case PTC5:
            if((SIM->SCGC5 & SIM_SCGC5_PORTC_MASK) == 0) SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;    // If PORTC is inactive: Enable
            PORTC->PCR[5] = PORT_PCR_MUX(6);                                                    // Set PTC5 mux to CMP0
            break;
        case PTE0:
            if((SIM->SCGC5 & SIM_SCGC5_PORTE_MASK) == 0) SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;    // If PORTE is inactive: Enable
            PORTE->PCR[0] = PORT_PCR_MUX(5);                                                    // Set PTE0 mux to CMP0
            break;
        default:
        break;
    }
}

void ComparatorIn::op_disable(PinName pdi)
{
    switch (pdi)
    {
        case PTC0:
            PORTC->PCR[0] &= PORT_PCR_MUX(1);             // Set PTC0 mux to ALT1
            break;
        case PTC5:
            PORTC->PCR[5] &= PORT_PCR_MUX(1);             // Set PTC5 mux to ALT1
            break;
        case PTE0:
            PORTE->PCR[0] &= PORT_PCR_MUX(1);             // Set PTE0 mux to ALT1
            break;
        default:
        break;
    }
}

void ComparatorIn::dac6_write(unsigned int value)
{
    unsigned int tmp;
    value &= 0x3F;                                   // 6-bit
    tmp = (CMP0->DACCR & 0xC0) | value;              // Replace old value
    CMP0->DACCR = tmp;                               // Set Vout DAC
}



