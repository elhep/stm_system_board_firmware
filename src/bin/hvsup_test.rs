//! ## Settings
//! Refer to the [Settings] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
#![deny(warnings)]
#![no_std]
#![no_main]
#![allow(unused)]

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
        devices::boards::Devices,
        ExtIntPin0,
        bus_manager::*
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
// use embedded_hal::blocking::delay::DelayMs;

//struct SysBoardTelemetry {
//    temp: u16,
//}

#[rtic::app(device = stm_sys_board::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    use stm_sys_board::hardware::devices::ic::max1329;
    use stm_sys_board::hardware::devices::ic::max1329::Max1329;
    use stm_sys_board::hardware::setup::SlotsBus;
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
            option_env!("BROKER")
                .unwrap_or("192.168.0.101")
                .parse()
                .unwrap(),
            Settings::default(),
        );

        let _i2c = stm_sys_board.therm_i2c;

        *c.local.bus_manager = Some(BusManager::new(stm_sys_board.slots_bus));
        let bus_manager = c.local.bus_manager.as_ref().unwrap();

        let mut device0 = Device0Type::new(5, bus_manager.acquire_bus());
        device0.init();

        let shared = Shared {
            network,
            device0,
            exti,
        };


        let local = Local {
            exti_pin0: exti_pins.0,
        };

        // settings_update::spawn().unwrap();
        telemetry0::spawn().unwrap();
        poll0::spawn().unwrap();
        ethernet_link::spawn().unwrap();

        (shared, local, init::Monotonics(stm_sys_board.systick))
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            //c.shared.ecp5.lock(|ecp| Max1329::set_adc_setup_direct(1,  ecp, max1329::adc::Mux::AVdd4_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar));

            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 2, shared=[network, device0])]
    fn settings_update(c: settings_update::Context) {
        let settings_update::SharedResources{
            mut device0, mut network
        } = c.shared;
        let settings = network.lock(|net| *net.miniconf.settings());

        match settings.device0_settings() {
            Some(dev_settings) => (device0).lock(|device| device.settings_update(dev_settings)),
            None => {},
        }

        // log::info!("SETTINGS UPDATE");
    }

    #[task(priority = 1, shared=[network, device0])]
    fn telemetry0(mut c: telemetry0::Context) {
        let (telemetry, telemetry_period) = c.shared.device0.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE0_TELEMETRY_PREFIX, &telemetry));
        telemetry0::Monotonic::spawn_after((telemetry_period as u64).millis())
            .unwrap();
    }

    #[task(priority = 1, shared=[network, device0])]
    fn poll0(mut c: poll0::Context) {
        let delay = c.shared.device0.lock(|dev| dev.poll());
        monotonics::now().ticks();
        if delay != 0 {
            poll0::Monotonic::spawn_after((delay as u64).millis()).unwrap();
        }
    }


    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
    }

    #[task(priority = 3, shared=[device0])]
    fn device0_check_interrupt(mut c: device0_check_interrupt::Context) {
        c.shared.device0.lock(|device| device.check_interrupt());
    }

    #[task(binds = EXTI3, priority = 4, local = [exti_pin0], shared = [exti])]
    fn device0interrupt(mut c: device0interrupt::Context) {
        log::info!(":::::::::::::::::::::::::::INTERRUPT:::::::::::::::::::::::::::::::");
        c.shared.exti.lock(|ex| {
            if ex.is_pending(Event::GPIO3) {
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

