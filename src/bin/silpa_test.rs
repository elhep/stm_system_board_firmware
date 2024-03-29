//! ## Settings
//! Refer to the [Settings] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
#![deny(warnings)]
#![no_std]
#![no_main]


// TODO delete all //// comments
// TODO Add P_PRES0_N and P_PRES1_P
// TODO Add devices PRSNT line check + exception if not found
// TODO Add device reset option + reset of main controller
// TODO powercycle - pin PS_ON_N (in DIOT it's software issue)

use fugit::ExtU64;

use stm_sys_board::{
    hardware::{
        self,
        hal,
        SystemTimer, Systick, ecp5::ECP5,
        devices::Devices,
        ExtIntPin0
    },
    net::{
        NetworkState, NetworkUsers,
    },
};
use stm32h7xx_hal::{gpio::ExtiPin,
                    exti::{Event, ExtiExt},
                    device::EXTI,};
//use core::option::Option::{self, Some};
use stm_sys_board::net::settings::{Settings, Device0Type,
                                               DEVICE0_TELEMETRY_PREFIX,};
use embedded_hal::blocking::delay::DelayMs;


//struct SysBoardTelemetry {
//    temp: u16,
//}

#[rtic::app(device = stm_sys_board::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    //use heapless::binary_heap::Max;
    use stm_sys_board::hardware::devices::max1329;
    use stm_sys_board::hardware::devices::max1329::Max1329;
    //use stm_sys_board::hardware::ecp5;
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings>,
        ecp5:    ECP5,
        device0: Device0Type,
        exti: EXTI,
    }

    #[local]
    struct Local {
        exti_pin0: ExtIntPin0,
        i2c: hal::i2c::I2c<hal::stm32::I2C1>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        let (stm_sys_board, exti, exti_pins) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
        );

        let mut ecp5 = stm_sys_board.ecp5;

        let network = NetworkUsers::new(
            stm_sys_board.net.stack,
            stm_sys_board.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            stm_sys_board.net.mac_address,
            option_env!("BROKER")
                .unwrap_or("192.168.0.101")
                .parse()
                .unwrap(),
            Settings::default(),
        );


        let mut i2c = stm_sys_board.therm_i2c;
        let mut i2c_bp = stm_sys_board.cpcis_i2c;
        let device0 = Device0Type::new(5);
        let mut servmod = stm_sys_board.servmod;
        let mut array : [u8; 2] = [0x00, 0x00];


        /*
            STM SYS BOARD - I2C temps sensors, EEPROM
        */

        log::info!("TEST 1: STM_SYS_Board I2C - temp sensors & eeprom");
        match hardware::lm75a::read_temp(&mut i2c, 0b1001_000){
            Ok(temp) => log::info!("Temp 1: {}", temp),
            Err(_e) => panic!("I2C 1st LM75 on Sys_Board error!"),
        };
        match hardware::lm75a::read_temp(&mut i2c, 0b1001_001){
            Ok(temp) => log::info!("Temp 2: {}", temp),
            Err(_e) => panic!("I2C 2nd LM75 on Sys_Board error!"),
        };
        hardware::eeprom::test_eeprom(&mut i2c, 0b1010_000).unwrap();

        log::info!("TEST SILPA SLOT 5: eeprom");
        servmod.4.set_low().unwrap();

        match hardware::lm75a::read_temp(&mut i2c_bp, 0b1001_000){
            Ok(temp) => log::info!("Temp 1: {}", temp),
            Err(_e) => panic!("I2C 1st LM75 on Sipla error!"),
        };
        match hardware::lm75a::read_temp(&mut i2c_bp, 0b1001_001){
            Ok(temp) => log::info!("Temp 2: {}", temp),
            Err(_e) => panic!("I2C 2nd LM75 on Sipla error!"),
        };
        hardware::eeprom::test_eeprom(&mut i2c_bp, 0b1010_000).unwrap();

        log::info!("Silpa I2C test done");
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ))  ;

        log::info!("Odbieranie ID od ECP5");
        delay.delay_ms(1000 as u32);
        ecp5.read_from_ecp5(40, &mut array).unwrap();
        log::info!("Odebrane id1: {} {}", array[0], array[1]);
        core::assert_eq!(array[0], 0xAA);
        core::assert_eq!(array[1], 0xAA);
        ecp5.read_from_ecp5(41, &mut array).unwrap();
        log::info!("Odebrane id2: {} {}", array[0], array[1]);
        core::assert_eq!(array[0], 0x55);
        core::assert_eq!(array[1], 0x55);

        log::info!("Odczyt inputów ze slotu 1");
        ecp5.read_inputs(1, &mut array);
        log::info!("Odebrane stany wejściowe: {} {}", array[0], array[1]);

        log::info!("Konfiguracja SPI dla slotu 2 (realnie 5)");
        Max1329::setup_ecp5_spi_master(1, &mut ecp5, 0);


        Max1329::reset_device(1, &mut ecp5);
        //delay.delay_ms(100000 as u32);
        log::info!("Próba odczytu SPI z MAX1329");
        let x = Max1329::read_clock_control_register(1, &mut ecp5);
        //delay.delay_ms(100000 as u32);
        log::info!("Clock control REG: {} - should be {}", x, 0b0110_0001);
        core::assert_eq!(x, 0b01100001);

        let x = Max1329::read_adc_gt_alarm_register(1, &mut ecp5);
        log::info!("MAIN GT ALARM {}", x);
        core::assert_eq!(x, 4095);

        log::info!("MAIN jESZCZE RAZ ODCZYT CLOCK CONTROL");
        let x = Max1329::read_clock_control_register(1, &mut ecp5);
        //delay.delay_ms(100000 as u32);
        log::info!("Clock control REG: {} - should be {}", x, 0b0110_0001);
        core::assert_eq!(x, 0b01100001);


        log::info!("MAIN ZAPIS DO INTERRUPT REGISTER");
        ecp5.check_registers();
        Max1329::set_interrupt_mask_register(1, &mut ecp5, 0xAABBCC);
        //delay.delay_ms(100000 as u32);
        log::info!("MAIN ODCZYT Z INTERRUPT REGISTER");
        // delay.delay_ms(100000 as u32);
        let x = Max1329::read_interrrupt_mask_register(1, &mut ecp5);
        //delay.delay_ms(100000 as u32);
        log::info!("main INT mask register {} {} {}", x[0], x[1], x[2]);
        core::assert_eq!(x[2], 0xCC);
        core::assert_eq!(x[1], 0xBB);
        core::assert_eq!(x[0], 0xAA);

        let x = Max1329::read_status_register(1, &mut ecp5);
        log::info!("Status register {}", x);
        //delay.delay_ms(100 as u32);
         let x = Max1329::read_status_register(1, &mut ecp5);
        log::info!("Status register drugi odczyt: {}", x);
        //delay.delay_ms(100 as u32);
        log::info!("Ustawienie PUMPa");


        struct Variables {
            pub cpvm_reg: u8,
            pub reference: max1329::adc::RefConf,
        }
        #[cfg(feature = "ext_ref_burned")]
        let variables = Variables{
            cpvm_reg: 0b0100_1001,
            reference: max1329::adc::RefConf::Int2_5,
        };

        #[cfg(not(feature = "ext_ref_burned"))]
        let variables = Variables{
            cpvm_reg: 0b0100_0001,
            reference: max1329::adc::RefConf::ExtBuffOff,
        };

        Max1329::set_cpvm_control_register(1, &mut ecp5, variables.cpvm_reg);

        #[allow(dead_code)]
        fn test_dac(ecp: &mut ECP5){
            log::info!("SET_DAC_CONTROL");
            Max1329::set_dac_control(1,  ecp,
                          max1329::dac::PowerDownConf::InOut,
                          max1329::dac::PowerDownConf::InOut,
                          max1329::dac::OpAmp::Disable,
                          max1329::dac::RefConf::Int2_5);
            log::info!("SET_DACA_VALUE");
            Max1329::set_daca_value(1, ecp, 0b0000_0110_0000_0000);
            Max1329::set_dacb_value(1, ecp, 0b0000_1000_0000_0000);

                    //delay.delay_ms(100 as u32);
            log::info!("STATUS READ:");
            let x = Max1329::read_status_register(1, ecp);
            log::info!("Status register {}", x);


            log::info!("DACA data READ");
            //delay.delay_ms(100 as u32);
            let x = Max1329::read_daca_value(1, ecp);
            log::info!("DAC A value {}", x);
            core::assert_eq!(x, 0b0000_0110_0000_0000);

            log::info!("DACB data READ");
            //delay.delay_ms(100 as u32);
            let x = Max1329::read_dacb_value(1, ecp);
            log::info!("DAC B value {}", x);
            core::assert_eq!(x, 0b0000_1000_0000_0000);
        }

        #[cfg(feature = "ext_ref_burned")]
        test_dac(&mut ecp5);

        //    -------------------------:::::::  ADC TESTS  :::::::-------------------------

        let x = Max1329::read_status_register(1, &mut ecp5);
        log::info!("Status przed ADC {}", x);

        delay.delay_ms(100 as u32);

        Max1329::set_interrupt_mask_register(1, &mut ecp5, 0b1110_1111_1111_1111_1111_1111); // unmask ADC done
        Max1329::set_adc_control_register(1, &mut ecp5, max1329::adc::AutoConversion::Disabled, max1329::adc::PowerDownConf::Normal, variables.reference);
        Max1329::set_adc_setup_register(1, &mut ecp5, max1329::adc::Mux::DVdd4_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        Max1329::set_adc_setup_direct(1, &mut ecp5, max1329::adc::Mux::DVdd4_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);


        while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {
            log::info!("W8 for ADC");
        }

        log::info!("Status po ADC {}", x);
        let x = Max1329::read_adc_data_register(1, &mut ecp5);

        log::info!("ADC value for DVDD {}", x.0);    // Dvdd / 4 (3.3 V / 4 = 0.825 V)

        Max1329::set_adc_setup_direct(1, &mut ecp5, max1329::adc::Mux::AVdd4_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {
            log::info!("W8 for ADC");
        }
        let x = Max1329::read_adc_data_register(1, &mut ecp5);
        log::info!("ADC value for AVDD {}", x.0);    // Avdd / 4 (4 V / 4 = 1 V)

        log::info!("ADC Control {}", Max1329::read_adc_control_register(1, &mut ecp5));
        log::info!("ADC Setup {}", Max1329::read_adc_setup_register(1, &mut ecp5));

        log::info!("HW_REV {}", Max1329::read_apio_setup_register(1, &mut ecp5) & 0x7);

        Max1329::set_dpio_control_register(1, &mut ecp5, 0xFFFF);
        Max1329::set_dpio_setup_register(1, &mut ecp5, 0x00);

        log::info!("DPIO SETUP {}", Max1329::read_dpio_setup_register(1, &mut ecp5));
        let x = Max1329::read_dpio_control_register(1, &mut ecp5);
        log::info!("DPIO CONTROL {} {}", x[0], x[1]);


        /*
            Check QSPI to ECP5 connection
        */

        // if device0.init(&mut ecp5){
        //     telemetry0::spawn().unwrap();
        // }

        let shared = Shared {
            network,
            ecp5,
            device0,
            exti,
        };


        let local = Local {
            exti_pin0: exti_pins.0,
            i2c,
        };

        settings_update::spawn().unwrap();
        ethernet_link::spawn().unwrap();

        (shared, local, init::Monotonics(stm_sys_board.systick))
    }

    #[idle(shared=[network], local=[i2c])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            //let mut data : [u8; 2] = [0; 2];
            //c.local.i2c.write_read(0b1001000 as u8, &[0], &mut data).unwrap();
            //let temp : u16 = ( (data[0] as u16) << 4) | ((data[1] as u16) >> 4);
            //log::info!("Temp: {}", (temp as f32) * 0.0625);

            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, shared=[network, ecp5, device0])]
    fn settings_update(c: settings_update::Context) {
        let settings_update::SharedResources{
            mut device0, mut ecp5, mut network
        } = c.shared;
        let settings = network.lock(|net| *net.miniconf.settings());

        (ecp5).lock(|ecp5| {
            match settings.device0_settings() {
                Some(dev_settings) => (device0).lock(|device| device.settings_update(ecp5, dev_settings)),
            None => {},
            }
        });
    }

    #[task(priority = 1, shared=[network, device0])]
    fn telemetry0(mut c: telemetry0::Context) {
        let (telemetry, telemetry_period) = c.shared.device0.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE0_TELEMETRY_PREFIX, &telemetry));

        telemetry0::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }


    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
    }

    #[task(priority = 3, shared=[device0, ecp5])]
    fn device0_check_interrupt(c: device0_check_interrupt::Context) {
        let device0_check_interrupt::SharedResources{
            device0, ecp5,
        } = c.shared;
        (device0, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(binds = EXTI15_10, priority = 4, local = [exti_pin0], shared = [exti])]
    fn device0interrupt(mut c: device0interrupt::Context) {
        c.shared.exti.lock(|ex| {
            if ex.is_pending(Event::GPIO13){
                c.local.exti_pin0.clear_interrupt_pending_bit();
                device0_check_interrupt::spawn().unwrap();
            }
        })
    }

//    #[task(binds = EXTI0, priority =5, local=[rst_per])]
//    fn system_reset(c: system_reset::Context) {
//        rst_per.set_low().unwrap();
//        // TODO RESET: Reset all settings and telemtry or just controler which should read devices registers?
//    }


    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }


}

