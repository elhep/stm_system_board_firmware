pub use miniconf::Miniconf;
use stm32h7xx_hal as hal;
use crate::hardware::ecp5::{self, ECP5};


pub mod adc;
pub mod dac;


// TODO to be implemented:
// - switch on/off DAC output
// - interrupt: eceeding threshold: currennt/voltage/power
// - telemtry for diagnostic data and min, max, avg. values



pub type SPIInterface = hal::xspi::Qspi<hal::device::QUADSPI>;


pub const WRITE :u8 = 0b00000000;
pub const READ :u8 = 0b00100000;

// Interrupts:
pub const VM1A : u32 = 1 << 23; // 1.8V DVdd
pub const VM1B : u32 = 1 << 22; // 2.7V DVdd
pub const VM2 : u32 = 1 << 21; // Avdd
pub const ADD : u32 = 1 << 20; // ADC Done
pub const AFF : u32 = 1 << 19; // ADC FIFO Full
pub const ACF : u32 = 1 << 18; // ADC Accumulator Full
pub const GTA : u32 = 1 << 17; // ADC GT Alarm
pub const LTA : u32 = 1 << 16; // ADC LT Alarm
pub const APR4 : u32 = 1 << 15; //  APIO4 Rising-Edge
pub const APR3 : u32 = 1 << 14; //  APIO3 Rising-Edge
pub const APR2 : u32 = 1 << 13; //  APIO2 Rising-Edge
pub const APR1 : u32 = 1 << 12; //  APIO1 Rising-Edge
pub const APF4 : u32 = 1 << 11; //  APIO4 Falling-Edge
pub const APF3 : u32 = 1 << 10; //  APIO3 Falling-Edge
pub const APF2 : u32 = 1 <<  9; //  APIO2 Falling-Edge
pub const APF1 : u32 = 1 <<  8; //  APIO1 Falling-Edge
pub const DPR4 : u32 = 1 <<  7; //  DPIO4 Rising-Edge
pub const DPR3 : u32 = 1 <<  6; //  DPIO3 Rising-Edge
pub const DPR2 : u32 = 1 <<  5; //  DPIO2 Rising-Edge
pub const DPR1 : u32 = 1 <<  4; //  DPIO1 Rising-Edge
pub const DPF4 : u32 = 1 <<  3; //  DPIO4 Falling-Edge
pub const DPF3 : u32 = 1 <<  2; //  DPIO3 Falling-Edge
pub const DPF2 : u32 = 1 <<  1; //  DPIO2 Falling-Edge
pub const DPF1 : u32 = 1      ; //  DPIO1 Falling-Edge


// Registers:
pub const ADC_CONTROL   :u8 = 0;
pub const ADC_SETUP     :u8 = 1;
pub const ADC_DATA      :u8 = 2;
pub const ADC_FIFO      :u8 = 3;
pub const ADC_ACC       :u8 = 4;
pub const ADC_GT_AL     :u8 = 5;
pub const ADC_LT_AL     :u8 = 6;
pub const DAC_CONTROL   :u8 = 7;
pub const FIFOA_CONTROL :u8 = 8;
pub const FIFOA_DATA    :u8 = 10;
pub const FIFO_SEQ      :u8 = 12;
pub const CLOCK_CONTROL :u8 = 13;
pub const CP_VM_CONTROL :u8 = 14;
pub const SWITCH_CONTROL:u8 = 15;
pub const APIO_CONTROL  :u8 = 16;
pub const APIO_SETUP    :u8 = 17;
pub const DPIO_CONTROL  :u8 = 18;
pub const DPIO_SETUP    :u8 = 19;
pub const STATUS        :u8 = 20;
pub const INTERRUPT_MASK:u8 = 21;
pub const RESET         :u8 = 31;


pub struct Max1329{
    slot: u8,
}


impl Max1329 {
    pub fn new(
        slot_number: u8,
    ) -> Self {
        Self {
            slot: slot_number,
        }
    }

    // Configure SPI in ECP5
    pub fn setup_ecp5_spi_master(slot: u8, ecp: &mut ECP5){
        let offset = slot * ecp5::OFFSET_TO_SLOT + ecp5::OFFSET_TO_SPI;
        ecp.write_to_ecp5(offset + ecp5::SPI::LENGTH, &mut [0x00, 0x0F]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::CS, &mut [0x00, 0x01]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::CS_POL, &mut [0x00, 0x00]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::DIV, &mut [0x00, 0x10]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::OFFLINE, &mut [0x00, 0x00]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::CLK_POL, &mut [0x00, 0x00]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::CLK_PHA, &mut [0x00, 0x00]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::LSB_FST, &mut [0x00, 0x00]).unwrap();
        ecp.write_to_ecp5(offset + ecp5::SPI::HALF_DUP, &mut [0x00, 0x00]).unwrap();
    }

    // INTERNAL REFERENCE ENABLE BIT is common for ADC and DAC
    pub fn set_adc_control_register(slot: u8,
                           ecp5: &mut ECP5,
                           auto: adc::AutoConversion,
                           apd: adc::PowerDownConf,
                           aref: adc::RefConf) {
        let data: u8;
        data = ((auto as u8) << 5) | ((apd as u8) << 3) | (aref as u8);
        let data = [ADC_CONTROL, data];
        ecp5.write_spi(slot, &data);
    }

// Direct commnand CAN NOT change MSEL
    pub fn set_adc_setup_direct(slot: u8,
                        ecp5: &mut ECP5,
                        mux: adc::Mux,
                        gain: adc::Gain,
                        bip: adc::Bip){
        let data: u8 = (1 << 7) | ( ((mux as u8) & 0xF)  << 3) | ((gain as u8) << 1) | (bip as u8);
        ecp5.write_spi(slot, &[data]);
    }

    pub fn set_adc_setup_register(slot: u8,
                         ecp5: &mut ECP5,
                         mux: adc::Mux,
                         gain: adc::Gain,
                         bip: adc::Bip) {
        let data: u8;
        data = ((mux as u8) << 3) | ((gain as u8) << 1) | (bip as u8);
        let data = [ADC_SETUP, data];
        ecp5.write_spi(slot, &data);
    }

    pub fn read_adc_data_register(slot: u8, ecp5: &mut ECP5) -> adc::AdcCode {
        let mut data: [u8; 2] = [0; 2];
        let address = [READ | ADC_DATA];
        ecp5.read_spi(slot, &address, &mut data);

        adc::AdcCode(((data[0] as u16) << 4) | ((data[1] as u16) >> 4))
    }

    pub fn read_adc_gt_alarm_register(slot: u8, ecp5: &mut ECP5) -> u16 {
        let mut data: [u8; 2] = [0; 2];
        let address = [READ | ADC_GT_AL];
        ecp5.read_spi(slot, &address, &mut data);
        log::info!("Odebrano: {} {}", data[0], data[1]);
        ((data[0] as u16) << 8) | data[1] as u16
    }



    pub fn set_adc_gt_alarm_register(slot: u8,
                                     ecp5: &mut ECP5,
                                     gtam: adc::AlarmMode,
                                     gtac: u8,
                                     gtat: u16) {
        let data: [u8; 3] = [ADC_GT_AL,
            ((gtam as u8) << 7) | (((gtac - 1) & 0x7) << 4) | (((gtat & 0xF00) >> 8) as u8),
            (gtat & 0xFF) as u8];
        ecp5.write_spi(slot, &data);
    }

    pub fn set_adc_lt_alarm_register(slot: u8,
                                     ecp5: &mut ECP5,
                                     ltam: adc::AlarmMode,
                                     ltac: u8,
                                     ltat: u16) {
        let data: [u8; 3] = [ADC_LT_AL,
            ((ltam as u8) << 7) | (((ltac - 1) & 0x7) << 4) | (((ltat & 0xF00) >> 8) as u8),
            (ltat & 0xFF) as u8];
        ecp5.write_spi(slot, &data);
    }


    pub fn set_dac_control(slot: u8,
                           ecp5: &mut ECP5,
                           dapd: dac::PowerDownConf,
                           dbpd: dac::PowerDownConf,
                           oa1e: dac::OpAmp,
                           dref: dac::RefConf) {
        let data: u8;
        data = ((dapd as u8) << 6) | ((dbpd as u8) << 4) | ((oa1e as u8) << 3) | (dref as u8);
        let data = [DAC_CONTROL, data];
        ecp5.write_spi(slot, &data);
    }

    pub fn set_dac_fifoa_control(self,
                                 ecp5: &mut ECP5,
                                 ffae: dac::FifoAEnable,
                                 bip: dac::Bip,
                                 syma: dac::Symmetry,
                                 cona: dac::Continuous,
                                 dpta: u8) { // 3 bits
        let data = ((ffae as u8) << 7) |
            ((bip as u8) << 6) |
            ((syma as u8) << 5) |
            ((cona as u8) << 4) |
            (dpta as u8);
        let data = [FIFOA_CONTROL, data];
        ecp5.write_spi(self.slot, &data);
    }

    pub fn set_daca_value(slot: u8,
                          ecp5: &mut ECP5,
                          value: u16){
        let data: [u8; 2] = [(0b0100 << 4) | ((value >> 8) as u8), (value & 0xFF) as u8];
        ecp5.write_spi(slot, &data);
    }

    pub fn set_dacb_value(slot: u8,
                      ecp5: &mut ECP5,
                      value: u16){
    let data: [u8; 2] = [(0b0101 << 4) | ((value >> 8) as u8), (value & 0xFF) as u8];
    ecp5.write_spi(slot, &data);
}

    pub fn set_dac_fifoa_data(self,
                              ecp5: &mut ECP5,
                              fifo_data: &[u16], // max 16 words
                              enable_after: bool,
    ) {
        // Disable FIFOa
        let mut control_reg: [u8; 1] = [0];
        let address : [u8; 1] = [READ | FIFOA_CONTROL];
        ecp5.read_spi(self.slot, &address, &mut control_reg);
        let mut control_reg=[0; 2];
        control_reg[0] = FIFOA_CONTROL;
        control_reg[1] = control_reg[1] & 0b0111_1111;
        ecp5.write_spi(self.slot, &control_reg);

        // Prepare data
        let mut data : [u8; 33] = [0; 33];
        data[0] = FIFOA_DATA;
        for (i, elem) in fifo_data.iter().enumerate(){
            data[2*i+1] = ((elem & 0xFF0) >> 4) as u8;
            data[2*i+2] = ((elem & 0x00F) << 4) as u8;
        }
        ecp5.write_spi(self.slot, &data);

        // Enable FIFOa
        if enable_after{
            control_reg[1] = control_reg[1] | 0b1000_0000;
            ecp5.write_spi(self.slot, &control_reg);
        }
    }

    pub fn set_dac_fifo_sequence(self,
                                 ecp5: &mut ECP5){
        let data: [u8; 2] = [FIFO_SEQ | WRITE, 0];
        ecp5.write_spi(self.slot, &data);
    }

    pub fn set_clock_control_register(slot : u8, ecp5: &mut ECP5, register_value : u8){
        let data : [u8; 2] = [CLOCK_CONTROL | WRITE, register_value];
        ecp5.write_spi(slot, &data);
    }

    pub fn read_clock_control_register(slot : u8, ecp5: &mut ECP5) -> u8 {
        let mut data: [u8; 1] = [0];

        log::info!("to write: {}", CLOCK_CONTROL | READ);
        ecp5.read_spi(slot, &[CLOCK_CONTROL | READ], &mut data);
        data[0]
    }

    pub fn set_cpvm_control_register(slot : u8, ecp5: &mut ECP5, register_value : u8){
        let data : [u8; 2] = [CP_VM_CONTROL | WRITE, register_value];
        ecp5.write_spi(slot, &data);
    }

    pub fn set_interrupt_mask_register(slot: u8,
                                       ecp5: &mut ECP5,
                                       register_value: u32){
        let mut data: [u8; 4] = [0; 4];
        data[0] = INTERRUPT_MASK | WRITE;
        data[1] = ((register_value & 0xFF0000) >> 16) as u8;
        data[2] = ((register_value & 0xFF00) >> 8) as u8;
        data[3] = (register_value & 0x00FF) as u8;

        log::info!("WRITE SPI Przygotowane do wysyłki: {} {} {} {}", data[0], data[1], data[2], data[3]);
        ecp5.write_spi(slot, &data);
    }

    pub fn read_interrrupt_mask_register(slot: u8,
                                         ecp5: &mut ECP5,
    ) -> [u8; 3] {
        let mut data: [u8; 3] = [0; 3];
        let address = [INTERRUPT_MASK | READ];
        ecp5.read_spi(slot, &address, &mut data);
        data
    }

    pub fn read_status_register(slot : u8,
                                ecp5: &mut ECP5,) -> u32 {
        let mut data: [u8; 3] = [0; 3];
        let address = [STATUS | READ];
        ecp5.read_spi(slot, &address, &mut data);

        ((data[0] as u32) << 16) |
        ((data[1] as u32) << 8) |
        (data[2] as u32)
    }

    pub fn read_apio_setupr_register(slot: u8,
                                     ecp5: &mut ECP5) -> u8 {
        let mut data : [u8; 1] = [0; 1];
        let address = [APIO_SETUP | READ];
        ecp5.read_spi(slot, &address, &mut data);

        data[0]
    }

    pub fn set_apio_control_register(slot : u8,
                                     ecp5: &mut ECP5,
                                     register_value: u8){
        let data : [u8; 2] = [APIO_CONTROL | WRITE, register_value];
        ecp5.write_spi(slot, &data);
    }

    pub fn set_dpio_control_register(slot : u8,
                                     ecp5: &mut ECP5,
                                     register_value: u16){
        let data :[u8; 3] = [DPIO_CONTROL | WRITE, (register_value >> 8) as u8, (register_value & 0xF) as u8];
        ecp5.write_spi(slot, &data);
    }

    pub fn set_dpio_setup_register(slot : u8,
                                     ecp5: &mut ECP5,
                                     register_value: u8){
        let data :[u8; 2] = [DPIO_SETUP | WRITE, register_value];
        ecp5.write_spi(slot, &data);
    }
}