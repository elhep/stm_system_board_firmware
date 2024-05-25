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
        SystemTimer, Systick,
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



//struct SysBoardTelemetry {
//    temp: u16,
//}

#[rtic::app(device = stm_sys_board::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    // use shared_bus::BusManager;
    use crate::hardware::bus_manager::BusManager;
    // use embedded_hal::digital::v2::OutputPin;
    use stm_sys_board::hardware::setup::SlotsBus;
    //use stm_sys_board::hardware::ecp5;
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings>,
        device0: Device0Type,
        exti: EXTI,
    }

    #[local]
    struct Local {
        exti_pin0: ExtIntPin0,
        // i2c: hal::i2c::I2c<hal::stm32::I2C1>,
    }

    #[init (local = [bus_manager: Option<BusManager<SlotsBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        let (stm_sys_board, exti, exti_pins) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
        );

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
                .unwrap_or("192.168.95.157") // Ustawic adres brokera ("127.0.0.1")
                .parse()
                .unwrap(),
            Settings::default(),
        );

        *c.local.bus_manager = Some(BusManager::new(stm_sys_board.slots_bus));
        let bus_manager = c.local.bus_manager.as_ref().unwrap();

        
        let _i2c = stm_sys_board.therm_i2c;
        let mut device0 = Device0Type::new(1, bus_manager.acquire_bus());

        device0.init();

        
        // let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
        //     400000000,
        // ));

        // delay.delay_ms(1000 as u32);
        
        // if device0.init(&mut ecp5){
        //     telemetry0::spawn().unwrap();
        // }

        let shared = Shared {
            network,
            device0,
            exti,
        };


        let local = Local {
            exti_pin0: exti_pins.0,
        };


        telemetry0::spawn().unwrap();
        // settings_update::spawn().unwrap();
        ethernet_link::spawn().unwrap();

        (shared, local, init::Monotonics(stm_sys_board.systick))
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }


        }
    }

    #[task(priority = 1, shared=[network, device0])]
    fn settings_update(c: settings_update::Context) {
        log::info!("----------- Settings Update --------------");
        let settings_update::SharedResources{
            mut device0, 
            mut network,
        } = c.shared;
        let settings = network.lock(|net| *net.miniconf.settings());

        match settings.device0_settings() {
            Some(dev_settings) => (device0).lock(|device| device.settings_update(dev_settings)),
            None => {},
        }
    }

    #[task(priority = 1, shared=[network, device0])]
    fn telemetry0(mut c: telemetry0::Context) {
        // log::info!("----------- Telemetry --------------");

        let (telemetry, telemetry_period) = c.shared.device0.lock(|device| 
            (
            device.telemetry())
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

    #[task(priority = 3, shared=[network, device0])]
    fn device0_check_interrupt(c: device0_check_interrupt::Context) {
        log::info!("------------ Interrupt Check --------------");
        let device0_check_interrupt::SharedResources{
            mut device0,
            mut network
        } = c.shared;

        
        let (telemetry , _) = (device0).lock(|device|
            (
                device.check_interrupt(),
                device.telemetry()
            )

        ); 
        
        log::info!("Telemetry inside interrupt");

        network.lock(|net| {
            net.telemetry.publish(DEVICE0_TELEMETRY_PREFIX, &telemetry);
            net.telemetry.update()
        });
        log::info!("------------ Interrupt Check Finished --------------");
        
        // (device0, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(binds = EXTI2, priority = 4, local = [exti_pin0], shared = [exti])]
    fn device0interrupt(mut c: device0interrupt::Context) {
        c.shared.exti.lock(|ex| {
            if ex.is_pending(Event::GPIO2){
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