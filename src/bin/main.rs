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
        ExtIntPin0, ExtIntPin1, ExtIntPin2,
        ExtIntPin3, ExtIntPin4, ExtIntPin5,
        ExtIntPin6,ExtIntPin7
    },
    net::{
        NetworkState, NetworkUsers,
    },
};
use stm32h7xx_hal::{gpio::ExtiPin,
                    exti::{Event, ExtiExt},
                    device::EXTI,};
use core::option::Option::{self, Some};
use stm_sys_board::net::settings::{Settings, Device0Type, Device1Type, Device2Type,
                                             Device3Type, Device4Type, Device5Type,
                                             Device6Type, Device7Type, DEVICE0_TELEMETRY_PREFIX,
                                             DEVICE1_TELEMETRY_PREFIX, DEVICE2_TELEMETRY_PREFIX,
                                             DEVICE3_TELEMETRY_PREFIX, DEVICE4_TELEMETRY_PREFIX,
                                             DEVICE5_TELEMETRY_PREFIX, DEVICE6_TELEMETRY_PREFIX,
                                             DEVICE7_TELEMETRY_PREFIX};
use hal::prelude::_embedded_hal_blocking_i2c_WriteRead;


//struct SysBoardTelemetry {
//    temp: u16,
//}

#[rtic::app(device = stm_sys_board::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings>,
        ecp5:    ECP5,
        device0: Device0Type,
        device1: Device1Type,
        device2: Device2Type,
        device3: Device3Type,
        device4: Device4Type,
        device5: Device5Type,
        device6: Device6Type,
        device7: Device7Type,
        exti: EXTI,
    }

    #[local]
    struct Local {
        exti_pin0: ExtIntPin0,
        exti_pin1: ExtIntPin1,
        exti_pin2: ExtIntPin2,
        exti_pin3: ExtIntPin3,
        exti_pin4: ExtIntPin4,
        exti_pin5: ExtIntPin5,
        exti_pin6: ExtIntPin6,
        exti_pin7: ExtIntPin7,
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
        let mut data : [u8; 2] = [0; 2];
        i2c.write_read(0b1001000 as u8, &[0], &mut data).unwrap();
        let temp : u16 = ( (data[0] as u16) << 4) | ((data[1] as u16) >> 4);
        log::info!("Temp: {}", (temp as f32) * 0.0625);

        let mut device0 = Device0Type::new(0);
        let mut device1 = Device1Type::new(1);
        let mut device2 = Device2Type::new(2);
        let mut device3 = Device3Type::new(3);
        let mut device4 = Device4Type::new(4);
        let mut device5 = Device5Type::new(5);
        let mut device6 = Device6Type::new(6);
        let mut device7 = Device7Type::new(7);

        if device0.init(&mut ecp5){
            telemetry0::spawn().unwrap();
        }
        if device1.init(&mut ecp5){
            telemetry1::spawn().unwrap();
        }
        if device2.init(&mut ecp5){
            telemetry2::spawn().unwrap();
        }
        if device3.init(&mut ecp5){
            telemetry3::spawn().unwrap();
        }
        if device4.init(&mut ecp5){
            telemetry4::spawn().unwrap();
        }
        if device5.init(&mut ecp5){
            telemetry5::spawn().unwrap();
        }
        if device6.init(&mut ecp5){
            telemetry6::spawn().unwrap();
        }
        if device7.init(&mut ecp5){
            telemetry7::spawn().unwrap();
        }

        let shared = Shared {
            network,
            ecp5,
            device0,
            device1,
            device2,
            device3,
            device4,
            device5,
            device6,
            device7,
            exti,
        };


        let local = Local {
            exti_pin0: exti_pins.0,
            exti_pin1: exti_pins.1,
            exti_pin2: exti_pins.2,
            exti_pin3: exti_pins.3,
            exti_pin4: exti_pins.4,
            exti_pin5: exti_pins.5,
            exti_pin6: exti_pins.6,
            exti_pin7: exti_pins.7,
            i2c,
        };

        settings_update::spawn().unwrap();
        ethernet_link::spawn().unwrap();

        (shared, local, init::Monotonics(stm_sys_board.systick))
    }

    #[idle(shared=[network], local=[i2c])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            let mut data : [u8; 2] = [0; 2];
            c.local.i2c.write_read(0b1001000 as u8, &[0], &mut data).unwrap();
            let temp : u16 = ( (data[0] as u16) << 4) | ((data[1] as u16) >> 4);
            log::info!("Temp: {}", (temp as f32) * 0.0625);

            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, shared=[network, ecp5, device0, device1, device2, device3, device4, device5, device6, device7])]
    fn settings_update(c: settings_update::Context) {
        let settings_update::SharedResources{
            mut device0, mut device1, mut device2, mut device3, mut device4, mut device5, mut device6, mut device7, mut ecp5, mut network
        } = c.shared;
        let settings = network.lock(|net| *net.miniconf.settings());

        (ecp5).lock(|ecp5| {
            match settings.device0_settings() {
                Some(dev_settings) => (device0).lock(|device| device.settings_update(ecp5, dev_settings)),
            None => {},
            }
            match settings.device1_settings() {
                Some(dev_settings) => (device1).lock(|device| device.settings_update(ecp5, dev_settings)),
                None => {},
            }
            match settings.device2_settings() {
                Some(dev_settings) => (device2).lock(|device| device.settings_update(ecp5, dev_settings)),
                None => {},
            }
            match settings.device3_settings() {
                Some(dev_settings) => (device3).lock(|device| device.settings_update(ecp5, dev_settings)),
                None => {},
            }
            match settings.device4_settings() {
                Some(dev_settings) => (device4).lock(|device| device.settings_update(ecp5, dev_settings)),
                None => {},
            }
            match settings.device5_settings() {
                Some(dev_settings) => (device5).lock(|device| device.settings_update(ecp5, dev_settings)),
                None => {},
            }
            match settings.device6_settings() {
                Some(dev_settings) => (device6).lock(|device| device.settings_update(ecp5, dev_settings)),
                None => {},
            }
            match settings.device7_settings() {
                Some(dev_settings) => (device7).lock(|device| device.settings_update(ecp5, dev_settings)),
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

    #[task(priority = 1, shared=[network, device1])]
    fn telemetry1(mut c: telemetry1::Context) {
        let (telemetry, telemetry_period) = c.shared.device1.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE1_TELEMETRY_PREFIX, &telemetry));

        telemetry1::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[network, device2])]
    fn telemetry2(mut c: telemetry2::Context) {
        let (telemetry, telemetry_period) = c.shared.device2.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE2_TELEMETRY_PREFIX, &telemetry));

        telemetry2::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[network, device3])]
    fn telemetry3(mut c: telemetry3::Context) {
        let (telemetry, telemetry_period) = c.shared.device3.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE3_TELEMETRY_PREFIX, &telemetry));

        telemetry3::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[network, device4])]
    fn telemetry4(mut c: telemetry4::Context) {
        let (telemetry, telemetry_period) = c.shared.device4.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE4_TELEMETRY_PREFIX, &telemetry));

        telemetry4::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[network, device5])]
    fn telemetry5(mut c: telemetry5::Context) {
        let (telemetry, telemetry_period) = c.shared.device5.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE5_TELEMETRY_PREFIX, &telemetry));

        telemetry5::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[network, device6])]
    fn telemetry6(mut c: telemetry6::Context) {
        let (telemetry, telemetry_period) = c.shared.device6.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE6_TELEMETRY_PREFIX, &telemetry));

        telemetry6::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[network, device7])]
    fn telemetry7(mut c: telemetry7::Context) {
        let (telemetry, telemetry_period) = c.shared.device7.lock(|device| (device.telemetry()));

        c.shared.network.lock(|net| net.telemetry.publish(DEVICE7_TELEMETRY_PREFIX, &telemetry));

        telemetry7::Monotonic::spawn_after((telemetry_period as u64).secs())
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

    #[task(priority = 3, shared=[device1, ecp5])]
    fn device1_check_interrupt(c: device1_check_interrupt::Context) {
        let device1_check_interrupt::SharedResources{
            device1, ecp5,
        } = c.shared;
        (device1, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(priority = 3, shared=[device2, ecp5])]
    fn device2_check_interrupt(c: device2_check_interrupt::Context) {
        let device2_check_interrupt::SharedResources{
            device2, ecp5,
        } = c.shared;
        (device2, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(priority = 3, shared=[device3, ecp5])]
    fn device3_check_interrupt(c: device3_check_interrupt::Context) {
        let device3_check_interrupt::SharedResources{
            device3, ecp5,
        } = c.shared;
        (device3, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(priority = 3, shared=[device4, ecp5])]
    fn device4_check_interrupt(c: device4_check_interrupt::Context) {
        let device4_check_interrupt::SharedResources{
            device4, ecp5,
        } = c.shared;
        (device4, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(priority = 3, shared=[device5, ecp5])]
    fn device5_check_interrupt(c: device5_check_interrupt::Context) {
        let device5_check_interrupt::SharedResources{
            device5, ecp5,
        } = c.shared;
        (device5, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(priority = 3, shared=[device6, ecp5])]
    fn device6_check_interrupt(c: device6_check_interrupt::Context) {
        let device6_check_interrupt::SharedResources{
            device6, ecp5,
        } = c.shared;
        (device6, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(priority = 3, shared=[device7, ecp5])]
    fn device7_check_interrupt(c: device7_check_interrupt::Context) {
        let device7_check_interrupt::SharedResources{
            device7, ecp5,
        } = c.shared;
        (device7, ecp5).lock(|device, ecp5| device.check_interrupt(ecp5));
    }

    #[task(binds = EXTI9_5, priority = 4, local=[exti_pin3, exti_pin4, exti_pin5], shared=[exti])]
    fn device3_5_interrupt(mut c: device3_5_interrupt::Context) {
        c.shared.exti.lock(|ex| {
            if ex.is_pending(Event::GPIO5) {
                c.local.exti_pin3.clear_interrupt_pending_bit();
                device3_check_interrupt::spawn().unwrap()
            }
            if ex.is_pending(Event::GPIO6) {
                c.local.exti_pin4.clear_interrupt_pending_bit();
                device4_check_interrupt::spawn().unwrap()
            }
            if ex.is_pending(Event::GPIO7) {
                c.local.exti_pin5.clear_interrupt_pending_bit();
                device5_check_interrupt::spawn().unwrap()
            }
        });

    }

    #[task(binds = EXTI15_10, priority = 4, local = [exti_pin0, exti_pin6, exti_pin7], shared = [exti])]
    fn device0_6_7_interrupt(mut c: device0_6_7_interrupt::Context) {
        c.shared.exti.lock(|ex| {
            if ex.is_pending(Event::GPIO11){
                c.local.exti_pin6.clear_interrupt_pending_bit();
                device6_check_interrupt::spawn().unwrap();
            }
            if ex.is_pending(Event::GPIO12){
                c.local.exti_pin7.clear_interrupt_pending_bit();
                device7_check_interrupt::spawn().unwrap();
            }
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

    #[task(binds = EXTI1, priority =4, local=[exti_pin1])]
    fn device1_interrupt(c: device1_interrupt::Context) {
        c.local.exti_pin1.clear_interrupt_pending_bit();
        device1_check_interrupt::spawn().unwrap()
    }

    #[task(binds = EXTI4, priority =4, local=[exti_pin2])]
    fn device2_interrupt(c: device2_interrupt::Context) {
        c.local.exti_pin2.clear_interrupt_pending_bit();
         device2_check_interrupt::spawn().unwrap()
    }


    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }


}

