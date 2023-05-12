pub use embedded_hal;
pub use stm32h7xx_hal as hal;
pub use hal::gpio::gpioa as gpioa;
pub use hal::gpio::gpiob as gpiob;
pub use hal::gpio::gpioc as gpioc;
pub use hal::gpio::gpiod as gpiod;
pub use hal::gpio::gpioe as gpioe;
pub use hal::gpio::gpiof as gpiof;
pub use hal::gpio::gpiog as gpiog;
pub mod devices;
pub mod design_parameters;
pub mod setup;
pub mod ecp5;
pub mod lm75a;

pub mod eeprom;
pub type InputFloating = hal::gpio::Input<hal::gpio::Floating>;
pub type InputPullUp = hal::gpio::Input<hal::gpio::PullUp>;
pub type InputPullDown = hal::gpio::Input<hal::gpio::PullDown>;
pub type OutputPushPull = hal::gpio::Output<hal::gpio::PushPull>;
pub type OutputOpenDrain = hal::gpio::Output<hal::gpio::OpenDrain>;


pub type ECP5InterfaceReady = (
    DigitalInput0Slot0,
    DigitalInput0Slot1,
    DigitalInput0Slot2,
    DigitalInput0Slot3,
    DigitalInput0Slot4,
    DigitalInput0Slot5,
    DigitalInput0Slot6,
    DigitalInput0Slot7,
);

pub type DeviceIO = (
     DigitalInput1Slot0,
     DigitalInput1Slot1,
     DigitalInput1Slot2,
     DigitalInput1Slot3,
     DigitalInput1Slot4,
     DigitalInput1Slot5,
     DigitalInput1Slot6,
     DigitalInput1Slot7
);

pub type ExtIntPins = (
    ExtIntPin0,
    ExtIntPin1,
    ExtIntPin2,
    ExtIntPin3,
    ExtIntPin4,
    ExtIntPin5,
    ExtIntPin6,
    ExtIntPin7,
);

pub type ExtIntPin0 = gpioc::PC3<InputPullUp>;
pub type ExtIntPin1 = gpiod::PD1<InputPullUp>;
pub type ExtIntPin2 = gpiog::PG4<InputPullUp>;
pub type ExtIntPin3 = gpiog::PG5<InputPullUp>;
pub type ExtIntPin4 = gpiog::PG6<InputPullUp>;
pub type ExtIntPin5 = gpiog::PG7<InputPullUp>;
pub type ExtIntPin6 = gpioe::PE11<InputPullUp>;
pub type ExtIntPin7 = gpioe::PE12<InputPullUp>;

pub type ServMod = (
    gpiof::PF2<OutputPushPull>,
    gpiof::PF3<OutputPushPull>,
    gpiof::PF4<OutputPushPull>,
    gpiof::PF5<OutputPushPull>,
    gpiof::PF12<OutputPushPull>,
    gpiof::PF13<OutputPushPull>,
    gpiof::PF14<OutputPushPull>,
    gpiof::PF15<OutputPushPull>,
);

// MonBus types:
mod mon_bus_types {
    use crate::hardware as top;
    pub type P_Pres = (top::gpiob::PB14<top::InputPullUp>,
                        top::gpioc::PC8<top::InputPullUp>);
    pub type P_RST = top::gpioc::PC13<top::OutputPushPull>;
    pub type F_RST = top::gpioc::PC12<top::OutputPushPull>;
    pub type P_IOs = (top::gpiob::PB6<top::InputFloating>,
                      top::gpiob::PB9<top::InputFloating>,
                      top::gpiob::PB10<top::InputFloating>);
    pub type F_IOs = (top::gpioc::PC14<top::InputFloating>,
                      top::gpioc::PC15<top::InputFloating>);
}

// FP Leds:
pub type FpLeds = (
    gpioa::PA3<OutputPushPull>,
    gpioc::PC6<OutputPushPull>,
    gpioc::PC7<OutputPushPull>,
    gpioa::PA0<OutputPushPull>,
);

// Type alias for digital inputs 0
pub type DigitalInput0Slot0 = gpiod::PD7<InputPullDown>;
pub type DigitalInput1Slot0 = gpiog::PG9<InputPullUp>;

// Type alias for digital inputs 1
pub type DigitalInput0Slot1 = gpiog::PG10<InputPullDown>;
pub type DigitalInput1Slot1 = gpiog::PG12<InputFloating>;

// Type alias for digital inputs 2
pub type DigitalInput0Slot2 = gpioc::PC2<InputPullDown>;
pub type DigitalInput1Slot2 = gpioc::PC3<InputFloating>;

// Type alias for digital inputs 3   //B12 change
pub type DigitalInput0Slot3 = gpiob::PB4<InputPullDown>;
pub type DigitalInput1Slot3 = gpiod::PD3<InputFloating>;

// Type alias for digital inputs 4
pub type DigitalInput0Slot4 = gpiob::PB2<InputPullDown>;
pub type DigitalInput1Slot4 = gpiob::PB4<InputFloating>;

// Type alias for digital inputs 5
pub type DigitalInput0Slot5 = gpioc::PC10<InputPullDown>;
pub type DigitalInput1Slot5 = gpioa::PA4<InputFloating>;

// Type alias for digital inputs 6
pub type DigitalInput0Slot6 = gpioe::PE2<InputPullDown>;
pub type DigitalInput1Slot6 = gpioe::PE4<InputFloating>;

// Type alias for digital inputs 7
pub type DigitalInput0Slot7 = gpioe::PE5<InputPullDown>;
pub type DigitalInput1Slot7 = gpioe::PE6<InputFloating>;

// Number of TX descriptors in the ethernet descriptor ring.
const TX_DESRING_CNT: usize = 4;

// Number of RX descriptors in the ethernet descriptor ring.
const RX_DESRING_CNT: usize = 4;

pub type NetworkStack = smoltcp_nal::NetworkStack<
    'static,
    hal::ethernet::EthernetDMA<'static, TX_DESRING_CNT, RX_DESRING_CNT>,
    SystemTimer,
>;

pub type NetworkManager = smoltcp_nal::shared::NetworkManager<
    'static,
    hal::ethernet::EthernetDMA<'static, TX_DESRING_CNT, RX_DESRING_CNT>,
    SystemTimer,
>;

pub type EthernetPhy = hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>;

/// System timer (RTIC Monotonic) tick frequency
pub const MONOTONIC_FREQUENCY: u32 = 1_000;
pub type Systick = systick_monotonic::Systick<MONOTONIC_FREQUENCY>;
pub type SystemTimer = mono_clock::MonoClock<u32, MONOTONIC_FREQUENCY>;

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::{
        fmt::Write,
        sync::atomic::{AtomicBool, Ordering},
    };
    use cortex_m::asm;
    use rtt_target::{ChannelMode, UpChannel};

    cortex_m::interrupt::disable();

    // Recursion protection
    static PANICKED: AtomicBool = AtomicBool::new(false);
    while PANICKED.load(Ordering::Relaxed) {
        asm::bkpt();
    }
    PANICKED.store(true, Ordering::Relaxed);

    // Turn on both red LEDs, FP_LED_1, FP_LED_3
    let gpiod = unsafe { &*hal::stm32::GPIOD::ptr() };
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high());

    // Analogous to panic-rtt-target
    if let Some(mut channel) = unsafe { UpChannel::conjure(0) } {
        channel.set_mode(ChannelMode::BlockIfFull);
        writeln!(channel, "{}", info).ok();
    }

    // Abort
    asm::udf();
    // Halt
    // loop { core::sync::atomic::compiler_fence(Ordering::SeqCst); }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[cortex_m_rt::exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
