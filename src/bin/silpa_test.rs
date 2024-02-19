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
// use heapless::String;
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

use hardware::lm75a::LM75_TEMPERATURE;


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
    use stm_sys_board::hardware::eeprom::SiLPADetector;
    use stm_sys_board::hardware::setup::BackPlaneI2C;
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
        back_plane: BackPlaneI2C,
        silpa_detector: SiLPADetector, 
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
            // option_env!("BROKER")
            // .unwrap_or("127.0.0.1") // Ustawic adres brokera ("127.0.0.1")
            // .parse()
            // .unwrap(),
            option_env!("BROKER")
                .unwrap_or("192.168.95.145") // Ustawic adres brokera ("127.0.0.1")
                .parse()
                .unwrap(),
            Settings::default(),
        );

        let _prefix = stm_sys_board::net::get_device_prefix(env!("CARGO_BIN_NAME"), stm_sys_board.net.mac_address);
        log::info!("Prefix: {}", _prefix);


        
        let i2c = stm_sys_board.therm_i2c;
        let mut i2c_bp = stm_sys_board.cpcis_i2c;
        let device0 = Device0Type::new(5);
        let mut servmod = stm_sys_board.servmod;
        let mut array : [u8; 2] = [0x00, 0x00];

        servmod.4.set_low().unwrap();

        hardware::eeprom::test_eeprom(&mut i2c_bp, 0b1010_000).unwrap();


        let mut detector_coefficients : [f32; 4] = [34.0, -34.71, 34.0, -34.71];
        hardware::eeprom::test_example_coefficients(&mut i2c_bp, 0b1010_000, &mut detector_coefficients);
        let mut silpa_detector = hardware::eeprom::SiLPADetector::new(detector_coefficients[0],
                                                                                detector_coefficients[1], 
                                                                                detector_coefficients[2], 
                                                                                detector_coefficients[3]);



        
        let mut back_plane = BackPlaneI2C{i2c: i2c_bp, servmod};

        log::info!("Ustawienie TOS w CH1: {}", device0.settings.channels_tos[0]);
        hardware::lm75a::set_tos(&mut back_plane.i2c, 0b1001_000, device0.settings.channels_tos[0]);
        hardware::lm75a::set_thyst(&mut back_plane.i2c, 0b1001_000, device0.settings.channels_thyst[0]);
        let tos = hardware::lm75a::read_tos(&mut back_plane.i2c, 0b1001_000);
        log::info!("Ustawiona wartość TOSw CH1 : {}", tos);

        silpa_detector.set_coefficients(device0.slot, &mut back_plane.i2c, &mut back_plane.servmod); // Po tej funkcji servmod jest w stanie high

        
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ));

        delay.delay_ms(1000 as u32);
        ecp5.read_from_ecp5(40, &mut array).unwrap();
        ecp5.read_from_ecp5(41, &mut array).unwrap();
        ecp5.read_inputs(1, &mut array);

        Max1329::setup_ecp5_spi_master(1, &mut ecp5, 0);


        Max1329::reset_device(1, &mut ecp5);
        delay.delay_ms(1000 as u32);

        // -----------------------------------------------------------------------
        // Clock Control Register
        Max1329::set_clock_control_register(1, &mut ecp5, 0b0100_0011);
        let _x = Max1329::read_clock_control_register(1, &mut ecp5);
        // -----------------------------------------------------------------------


        // -----------------------------------------------------------------------
        // GT Alarm
        Max1329::set_adc_gt_alarm_register(1, &mut ecp5, max1329::adc::AlarmMode::Consecutive, 8, 1000);
        let _x = Max1329::read_adc_gt_alarm_register(1, &mut ecp5);
        // 1000 jest ustawione jako wartosc testowa
        // -----------------------------------------------------------------------

        // -----------------------------------------------------------------------
        // LT Alarm
        Max1329::set_adc_lt_alarm_register(1, &mut ecp5, max1329::adc::AlarmMode::Consecutive, 8, 500);
        let _x = Max1329::read_adc_lt_alarm_register(1, &mut ecp5);
        // -----------------------------------------------------------------------

        // -----------------------------------------------------------------------
        // CPVM Control
        // struct Variables {
        //     pub cpvm_reg: u8,
        //     _reference: max1329::adc::RefConf,
        // }
        // #[cfg(feature = "ext_ref_burned")]
        // let variables = Variables{
        //     cpvm_reg: 0b0100_1001,
        //     _reference: max1329::adc::RefConf::Int2_5,
        // };

        // #[cfg(not(feature = "ext_ref_burned"))]
        // let variables = Variables{
        //     cpvm_reg: 0b0100_0001,
        //     _reference: max1329::adc::RefConf::ExtBuffOff,
        // };

        log::info!("Ustawiana wartosc cpvm_reg: {}", 0b0100_1001);
        Max1329::set_cpvm_control_register(1, &mut ecp5, 0b0100_1001);
        // -----------------------------------------------------------------------

        #[allow(dead_code)]
        fn test_dac(ecp: &mut ECP5){
            Max1329::set_dac_control(1,  ecp,
                          max1329::dac::PowerDownConf::InOut,
                          max1329::dac::PowerDownConf::InOut,
                          max1329::dac::OpAmp::Disable,
                          max1329::dac::RefConf::Int2_5);

            Max1329::set_daca_value(1, ecp, 0b0000_1000_1111_0101); // 1.4 V na wyjsciu DAC, -2 dBm na wejsciu detektora,
            Max1329::set_dacb_value(1, ecp, 0b0000_0110_0000_0000);
            log::info!("Ustawiono wartosc DAC");

        }

        // 0.08V Threshold -> ~-10dBm + 26 dB = 16 dBm output
        // Warosc u16 = 0b0000_0000_1000_0011

        let x = Max1329::read_daca_value(1, & mut ecp5);
        log::info!("Wartosc daca przed test dac: {}", x);

        // #[cfg(feature = "ext_ref_burned")]
        test_dac(&mut ecp5);

        let x = Max1329::read_daca_value(1, & mut ecp5);
        log::info!("Wartosc daca po test dac: {}", x);


        ecp5.read_oe(1, &mut array);
        log::info!("Odczyt OE: {} {}", array[1], array[0]);

        let mut outputs_value : [u8; 2] = [0, 128];
        let mut outputs_enable : [u8; 2] = [0, 192];

        ecp5.write_oe(1, &mut outputs_enable);
        ecp5.write_outputs(1, &mut outputs_value);


        // Piny, bity do write_output, read_output
        //  4 - input, przerwanie z kanalu pierwszego
        //  5 - input, przerwanie z kanalu drugiego
        //  6 - output, resetowanie kanalu pierwszego
        //  7 - output, resetowanie kanalu pierwszego


        outputs_value = [0, 0];
        ecp5.write_outputs(1, &mut outputs_value);

        ecp5.read_interrupts_mask(1, &mut array);
        log::info!("Odczyt interrupt mask: {} {}", array[1], array[0]);

        // let mut interrupt_mask : [u8; 2] = [48, 0];
        // ecp5.write_interrupts_mask(1, &mut interrupt_mask);



        //    -------------------------:::::::  ADC TESTS  :::::::-------------------------

        // Do testow ustawiony odczyt napiecia DVDD w ogolnosci MSEL = 0 i wartosci 0 oraz 1 tak aby miec odczyt AIN1 - AGND oraz AIN2 - AGND

        Max1329::set_interrupt_mask_register(1, &mut ecp5, 0b1110_1100_1111_1111_1111_1111); // unmask ADC done, GT, LT
        Max1329::set_adc_control_register(1, &mut ecp5, max1329::adc::AutoConversion::Disabled, max1329::adc::PowerDownConf::Normal, max1329::adc::RefConf::Int2_5);
        
        Max1329::set_adc_setup_direct(1, &mut ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        
        
        while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {
            log::info!("W8 for ADC");
        }
        let x = Max1329::read_adc_data_register(1, &mut ecp5);
        log::info!("ADC value for AIN1_GND {}", x.0);  

        // -----------------------------------------------------------------------   
        // log::info!("Test ADC ma AVDD");
        // Max1329::set_adc_setup_direct(1, &mut ecp5, max1329::adc::Mux::AVdd4_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        // while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {};
        // let x = Max1329::read_adc_data_register(1, &mut ecp5);
        // log::info!("ADC value for AVDD {}", x.0);  

        // Wlaczenie odpowiednich odczytow miedzy AIN1-AGND i AIN2-AGND, odczekiwanie na koniec konwersji i odczyt wartosci z rejestru 
        // REFE = 1 wlacza internal reference
        // Max1329::set_adc_control_register(1, &mut ecp5, max1329::adc::AutoConversion::Disabled, max1329::adc::PowerDownConf::Normal, max1329::adc::RefConf::Int2_5);
        Max1329::set_adc_setup_direct(1, &mut ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {
        }
        let x = Max1329::read_adc_data_register(1, &mut ecp5);
        log::info!("ADC value for AIN1_GND {}", x.0); 

        Max1329::set_adc_setup_direct(1, &mut ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, &mut ecp5) | (1 << 20)) == 0 {
        }
        let x = Max1329::read_adc_data_register(1, &mut ecp5);
        log::info!("ADC value for AIN2_GND {}", x.0); 
        // -----------------------------------------------------------------------

        Max1329::set_dpio_control_register(1, &mut ecp5, 0xFFFF); // Ustawienie DPIO jako output, zmiany wartosci trzeba wpisac do DP_LL
        Max1329::set_dpio_setup_register(1, &mut ecp5, 0x03); // Ustawienie wewnetrznych pullopow i wlaczenie wszystkich tranzystorow na start
        
        // if device0.init(&mut ecp5){
        //     telemetry0::spawn().unwrap();
        // }

        let shared = Shared {
            network,
            ecp5,
            device0,
            exti,
            back_plane,
            silpa_detector,
        };


        let local = Local {
            exti_pin0: exti_pins.0,
            i2c,
        };


        telemetry0::spawn().unwrap();
        // settings_update::spawn().unwrap();
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
            // log::info!("Idle");
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }


        }
    }

    #[task(priority = 1, shared=[network, ecp5, device0, silpa_detector, back_plane])]
    fn settings_update(c: settings_update::Context) {
        log::info!("----------- Settings Update --------------");
        let settings_update::SharedResources{
            device0, 
            mut ecp5, 
            mut network,
            silpa_detector,
            back_plane
        } = c.shared;
        let settings = network.lock(|net| *net.miniconf.settings());

        (ecp5).lock(|ecp5| {
            match settings.device0_settings() {
                Some(dev_settings) => {(device0, silpa_detector, back_plane).lock(|device0, silpa_detector, back_plane| (
                    device0.settings_update(ecp5, dev_settings),
                    if device0.settings.dacs_value[0] != dev_settings.dacs_value[0] {
                        log::info!("Zmiana wartosci Threshold CH1: {}", dev_settings.dacs_value[0]);
                        device0.calculate_dac_value(silpa_detector.channel_1_slope, silpa_detector.channel_1_intercept, dev_settings.dacs_value[0], 1, ecp5)
                    },

                    if device0.settings.dacs_value[1] != dev_settings.dacs_value[1] {
                        log::info!("Zmiana wartosci Threshold CH2: {}", dev_settings.dacs_value[1]);
                        device0.calculate_dac_value(silpa_detector.channel_1_slope, silpa_detector.channel_1_intercept, dev_settings.dacs_value[1], 2, ecp5)
                    },

                    if device0.settings.channels_tos[0] != dev_settings.channels_tos[0]{
                        log::info!("Zmiana wartosci TOS CH1: {}", dev_settings.channels_tos[0]);
                        hardware::lm75a::set_tos(&mut back_plane.i2c, hardware::lm75a::I2C_ADDR[0], dev_settings.channels_tos[0])
                    },

                    if device0.settings.channels_tos[1] != dev_settings.channels_tos[1]{
                        log::info!("Zmiana wartosci TOS CH2: {}", dev_settings.channels_tos[1]);
                        hardware::lm75a::set_tos(&mut back_plane.i2c, hardware::lm75a::I2C_ADDR[1], dev_settings.channels_tos[1])
                    },

                    if device0.settings.channels_thyst[0] != dev_settings.channels_thyst[0]{
                        log::info!("Zmiana wartosci THYST CH1: {}", dev_settings.channels_thyst[0]);
                        hardware::lm75a::set_thyst(&mut back_plane.i2c, hardware::lm75a::I2C_ADDR[0], dev_settings.channels_thyst[0])
                    },

                    if device0.settings.channels_thyst[1] != dev_settings.channels_thyst[1]{
                        log::info!("Zmiana wartosci THYST CH2: {}", dev_settings.channels_thyst[1]);
                        hardware::lm75a::set_thyst(&mut back_plane.i2c, hardware::lm75a::I2C_ADDR[1], dev_settings.channels_thyst[1])
                    },
                    
                ))},
                None => {((), (), (), (), (), (), ())},
            }
        });
    }

    #[task(priority = 1, shared=[network, ecp5, device0, back_plane])]
    fn telemetry0(mut c: telemetry0::Context) {
        log::info!("----------- Telemetry --------------");

        let (temperature_ch1, temperature_ch2) = c.shared.back_plane.lock(|back_plane| c.shared.device0.lock(|device| (
            (device.check_temperature(&mut back_plane.i2c, &mut back_plane.servmod, 1),
            device.check_temperature(&mut back_plane.i2c, &mut back_plane.servmod, 2))
        )));

        log::info!("Temp CH1 : {}", temperature_ch1);
        log::info!("Temp CH2 : {}", temperature_ch2);

        let (telemetry, telemetry_period) = c.shared.ecp5.lock(|ecp5| c.shared.device0.lock(|device| 
            (
            device.telemetry(ecp5)))
        );

        c.shared.network.lock(|net| {
            net.telemetry.publish(DEVICE0_TELEMETRY_PREFIX, &telemetry);
            net.telemetry.update();
        });

        telemetry0::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }


    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
    }

    #[task(priority = 3, shared=[device0, ecp5, back_plane])]
    fn device0_check_interrupt(c: device0_check_interrupt::Context) {
        log::info!("------------ Interrupt Check --------------");
        let device0_check_interrupt::SharedResources{
            device0, ecp5, back_plane
        } = c.shared;

        
        (back_plane, ecp5, device0).lock(|back_plane, ecp5, device|
            (
                if device.check_temperature(&mut back_plane.i2c, &mut back_plane.servmod, 1) > LM75_TEMPERATURE::TRESHOLD_CH1 {
                    device.settings.channels_locked[0] = true;
                    log::info!("Przekroczenie temperatury CH1");
                },

                if device.check_temperature(&mut back_plane.i2c, &mut back_plane.servmod, 2) > LM75_TEMPERATURE::TRESHOLD_CH2 {
                    device.settings.channels_locked[1] = true;
                    log::info!("Przekroczenie temperatury CH2");
                },

                device.check_interrupt(ecp5)
            )

        ); 

        
        // (device0, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
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