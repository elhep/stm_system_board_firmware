//! ## Settings
//! Refer to the [Settings] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
#![deny(warnings)]
#![no_std]
#![no_main]

// TODO
// Poczytac o tym jak to jest z MQTT i dopytac
// Ogarnac wysylanie/odbieranie po MQTT i tworzenie topicow

// TODO delete all //// comments
// TODO Add P_PRES0_N and P_PRES1_P
// TODO Add devices PRSNT line check + exception if not found
// TODO Add device reset option + reset of main controller
// TODO powercycle - pin PS_ON_N (in DIOT it's software issue)

use fugit::ExtU64;
use heapless::String;
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
    // use stm_sys_board::hardware::devices::max1329::adc;
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
                .unwrap_or("192.168.0.101") // Ustawic adres brokera ("127.0.0.1")
                .parse()
                .unwrap(),
            Settings::default(),
        );


        let mut i2c = stm_sys_board.therm_i2c;
        let mut i2c_bp = stm_sys_board.cpcis_i2c;
        let device0 = Device0Type::new(5);
        let mut servmod = stm_sys_board.servmod;
        let mut array : [u8; 2] = [0x00, 0x00];

        match network.miniconf.
        /*
            STM SYS BOARD - I2C temps sensors, EEPROM
        */

        log::info!("TEST 1: STM_SYS_Board I2C - temp sensors & eeprom");
        match hardware::lm75a::read_temp(&mut i2c, 0b1001_000){ // Addr 0x48
            Ok(temp) => log::info!("Temp 1: {}", temp),
            Err(_e) => panic!("I2C 1st LM75 on Sys_Board error!"),
        };
        match hardware::lm75a::read_temp(&mut i2c, 0b1001_001){ // Addr 0x49
            Ok(temp) => log::info!("Temp 2: {}", temp),
            Err(_e) => panic!("I2C 2nd LM75 on Sys_Board error!"),
        };
        hardware::eeprom::test_eeprom(&mut i2c, 0b1010_000).unwrap();

        log::info!("TEST SILPA SLOT 5: eeprom");
        servmod.4.set_low().unwrap();

        // match hardware::lm75a::read_temp(&mut i2c_bp, 0b1001_000){
        //     Ok(temp) => log::info!("Temp 1: {}", temp),
        //     Err(_e) => panic!("I2C 1st LM75 on Sipla error!"),
        // };
        // match hardware::lm75a::read_temp(&mut i2c_bp, 0b1001_001){
        //     Ok(temp) => log::info!("Temp 2: {}", temp),
        //     Err(_e) => panic!("I2C 2nd LM75 on Sipla error!"),
        // };
        hardware::eeprom::test_eeprom(&mut i2c_bp, 0b1010_000).unwrap();

        log::info!("Silpa I2C test done");
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ))  ;

        log::info!("Odbieranie ID od ECP5");
        delay.delay_ms(1000 as u32);
        ecp5.read_from_ecp5(40, &mut array).unwrap();
        log::info!("Odebrane id1: {} {}", array[0], array[1]);
        ecp5.read_from_ecp5(41, &mut array).unwrap();
        log::info!("Odebrane id2: {} {}", array[0], array[1]);

        log::info!("Odczyt inputów ze slotu 1");
        ecp5.read_inputs(1, &mut array);
        log::info!("Odebrane stany wejściowe: {} {}", array[0], array[1]);

        log::info!("Konfiguracja SPI dla slotu 2 (realnie 5)");
        Max1329::setup_ecp5_spi_master(1, &mut ecp5, 0);


        Max1329::reset_device(1, &mut ecp5);
        //delay.delay_ms(100000 as u32);

        // -----------------------------------------------------------------------
        // Clock Control
        log::info!("Próba Zapisu do Clock Control Register");
        Max1329::set_clock_control_register(1, &mut ecp5, 0b0100_0011);
        log::info!("Potwierdzenie zapisania poprawnej wartości");
        let x = Max1329::read_clock_control_register(1, &mut ecp5);
        log::info!("Clock control REG: {} - should be {}", x, 0b0100_0011);
        // -----------------------------------------------------------------------


        // -----------------------------------------------------------------------
        // GT Alarm
        log::info!("Próba Zapisu do GT Alarm Register");
        Max1329::set_adc_gt_alarm_register(1, &mut ecp5, max1329::adc::AlarmMode::Consecutive, 8, 1000);
        log::info!("Potwierdzenie zapisania poprawnej wartości");
        let x = Max1329::read_adc_gt_alarm_register(1, &mut ecp5);
        log::info!("GT Alarm REG: {} - should be {}", x, 62440);
        // 1000 jest ustawione jako wartosc testowa
        // -----------------------------------------------------------------------

        // -----------------------------------------------------------------------
        // LT Alarm
        log::info!("Próba Zapisu do LT Alarm Register");
        Max1329::set_adc_lt_alarm_register(1, &mut ecp5, max1329::adc::AlarmMode::Consecutive, 8, 500);
        log::info!("Potwierdzenie zapisania poprawnej wartości");
        let x = Max1329::read_adc_lt_alarm_register(1, &mut ecp5);
        log::info!("LT Alarm REG: {} - should be {}", x, 61940);
        // 1000 jest ustawione jako wartosc testowa
        // -----------------------------------------------------------------------

        // -----------------------------------------------------------------------
        // CPVM Control
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
        // -----------------------------------------------------------------------

        #[allow(dead_code)]
        fn test_dac(ecp: &mut ECP5){
            Max1329::set_dac_control(1,  ecp,
                          max1329::dac::PowerDownConf::InOut,
                          max1329::dac::PowerDownConf::InOut,
                          max1329::dac::OpAmp::Disable,
                          max1329::dac::RefConf::Int2_5);

            Max1329::set_daca_value(1, ecp, 0b0000_0110_0000_0000);
            Max1329::set_dacb_value(1, ecp, 0b0000_1000_0000_0000);

        }

        #[cfg(feature = "ext_ref_burned")]
        test_dac(&mut ecp5);

        //    -------------------------:::::::  ADC TESTS  :::::::-------------------------

        // Do testow ustawiony odczyt napiecia DVDD w ogolnosci MSEL = 0 i wartosci 0 oraz 1 tak aby miec odczyt AIN1 - AGND oraz AIN2 - AGND

        Max1329::set_interrupt_mask_register(1, &mut ecp5, 0b1110_1100_1111_1111_1111_1111); // unmask ADC done, GT, LT

        // -----------------------------------------------------------------------   
        // Wlaczenie odpowiednich odczytow miedzy AIN1-AGND i AIN2-AGND, odczekiwanie na koniec konwersji i odczyt wartosci z rejestru 
        // REFE = 1 wlacza internal reference
        Max1329::set_adc_control_register(1, &mut ecp5, max1329::adc::AutoConversion::Disabled, max1329::adc::PowerDownConf::Normal, variables.reference);
        Max1329::set_adc_setup_register(1, &mut ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {
            log::info!("W8 for ADC");
        }
        // let x = Max1329::read_adc_data_register(1, &mut ecp5);
        Max1329::set_adc_setup_register(1, &mut ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {
            log::info!("W8 for ADC");
        }
        // let x = Max1329::read_adc_data_register(1, &mut ecp5);
        // -----------------------------------------------------------------------

        Max1329::set_dpio_control_register(1, &mut ecp5, 0xFFFF); // Ustawienie DPIO jako output, zmiany wartosci trzeba wpisac do DP_LL
        Max1329::set_dpio_setup_register(1, &mut ecp5, 0x00); // Ustawienie wewnetrznych pullopow i wlaczenie wszystkich tranzystorow na start
        
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
        telemetry0::spawn().unwrap();

        (shared, local, init::Monotonics(stm_sys_board.systick))
    }

    #[idle(shared=[network], local=[i2c])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            //let mut data : [u8; 2] = [0; 2];
            //c.local.i2c.write_read(0b1001000 as u8, &[0], &mut data).unwrap();
            //let temp : u16 = ( (data[0] as u16) << 4) | ((data[1] as u16) >> 4);
            //log::info!("Temp: {}", (temp as f32) * 0.0625);
            log::info!("Idle");
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
        log::info!("Settings Update");
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
        log::info!("Telemetry");
        let (mut telemetry, telemetry_period) = c.shared.device0.lock(|device| (device.telemetry()));


        c.shared.network.lock(|net| {
            net.telemetry.publish(DEVICE0_TELEMETRY_PREFIX, &telemetry);
            telemetry.set_adc_val(20);
            net.telemetry.update();
            net.telemetry.publish(DEVICE0_TELEMETRY_PREFIX, &telemetry)});

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

