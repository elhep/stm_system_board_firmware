///! Sys_Board hardware configuration
///!
///! This file contains all of the hardware-specific configuration of Sys_Board.
use core::sync::atomic::{self, AtomicBool, Ordering};
use core::{ptr, slice};
use stm32h7xx_hal::{
    self as hal,
    ethernet::{self, PHY},
    prelude::*,
    time::MegaHertz,
    gpio::{ExtiPin, Edge},
};

use smoltcp_nal::smoltcp;

use embedded_hal::digital::v2::{OutputPin, InputPin};
use crate::hardware::DeviceIO;
use crate::hardware::ecp5::ECP5;

use super::{
    design_parameters, eeprom,
    EthernetPhy, NetworkStack, SystemTimer, Systick, ExtIntPins, mon_bus_types as MonBusTypes,
    ServMod
};

const NUM_TCP_SOCKETS: usize = 2;
const NUM_UDP_SOCKETS: usize = 1;
const NUM_SOCKETS: usize = NUM_UDP_SOCKETS + NUM_TCP_SOCKETS;

pub struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],

    // Note: There is an additional socket set item required for the DHCP socket.
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_SOCKETS + 1],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub udp_socket_storage: [UdpSocketStorage; NUM_UDP_SOCKETS],
    pub neighbor_cache:
        [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache:
        [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
}

#[derive(Copy, Clone)]
pub struct UdpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 2048],
    tx_metadata:
        [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
    rx_metadata:
        [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
}

impl UdpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 2048],
            tx_metadata: [smoltcp::storage::PacketMetadata::<
                smoltcp::wire::IpEndpoint,
            >::EMPTY; 10],
            rx_metadata: [smoltcp::storage::PacketMetadata::<
                smoltcp::wire::IpEndpoint,
            >::EMPTY; 10],
        }
    }
}

#[derive(Copy, Clone)]
pub struct TcpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 1024],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 1024],
        }
    }
}

impl Default for NetStorage {
    fn default() -> Self {
        NetStorage {
            // Placeholder for the real IP address, which is initialized at runtime.
            ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
                smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
            )],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [smoltcp::iface::SocketStorage::EMPTY; NUM_SOCKETS + 1],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
            udp_socket_storage: [UdpSocketStorage::new(); NUM_UDP_SOCKETS],
        }
    }
}

/// The available networking devices on Sys_Board.
pub struct NetworkDevices {
    pub stack: NetworkStack,
    pub phy: EthernetPhy,
    pub mac_address: smoltcp::wire::EthernetAddress,
}

/// The available hardware interfaces on Sys_Board.
pub struct BoardDevices {
    pub systick: Systick,
    pub net: NetworkDevices,
    //pub devicesio: DeviceIO,
    pub ecp5: ECP5,
    pub mon_bus: MonBus,
    pub therm_i2c: hal::i2c::I2c<hal::stm32::I2C1>,
    pub cpcis_i2c: hal::i2c::I2c<hal::stm32::I2C4>,
    pub servmod: ServMod,
}

pub struct MonBus {
    i2c: hal::i2c::I2c<hal::stm32::I2C2>,
    p_pres: MonBusTypes::P_Pres,
    p_rst: MonBusTypes::P_RST,
    f_rst: MonBusTypes::F_RST,
    p_ios: MonBusTypes::P_IOs,
    f_ios: MonBusTypes::F_IOs,
}


#[link_section = ".sram3.eth"]
/// Static storage for the ethernet DMA descriptor ring.
static mut DES_RING: ethernet::DesRing<
    { super::TX_DESRING_CNT },
    { super::RX_DESRING_CNT },
> = ethernet::DesRing::new();

/// Setup ITCM and load its code from flash.
///
/// For portability and maintainability this is implemented in Rust.
/// Since this is implemented in Rust the compiler may assume that bss and data are set
/// up already. There is no easy way to ensure this implementation will never need bss
/// or data. Hence we can't safely run this as the cortex-m-rt `pre_init` hook before
/// bss/data is setup.
///
/// Calling (through IRQ or directly) any code in ITCM before having called
/// this method is undefined.
fn load_itcm() {
    extern "C" {
        static mut __sitcm: u32;
        static mut __eitcm: u32;
        static mut __siitcm: u32;
    }
    // NOTE(unsafe): Assuming the address symbols from the linker as well as
    // the source instruction data are all valid, this is safe as it only
    // copies linker-prepared data to where the code expects it to be.
    // Calling it multiple times is safe as well.

    unsafe {
        // ITCM is enabled on reset on our CPU but might not be on others.
        // Keep for completeness.
        const ITCMCR: *mut u32 = 0xE000_EF90usize as _;
        ptr::write_volatile(ITCMCR, ptr::read_volatile(ITCMCR) | 1);

        // Ensure ITCM is enabled before loading.
        atomic::fence(Ordering::SeqCst);

        let len =
            (&__eitcm as *const u32).offset_from(&__sitcm as *const _) as usize;
        let dst = slice::from_raw_parts_mut(&mut __sitcm as *mut _, len);
        let src = slice::from_raw_parts(&__siitcm as *const _, len);
        // Load code into ITCM.
        dst.copy_from_slice(src);
    }

    // Ensure ITCM is loaded before potentially executing any instructions from it.
    atomic::fence(Ordering::SeqCst);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

fn create_interrupt_conf<T>(mut pin: T, exti: &mut hal::device::EXTI, syscfg: &mut hal::device::SYSCFG) -> T
    where
        T: ExtiPin
    {
        pin.make_interrupt_source(syscfg);
        pin.trigger_on_edge(exti, Edge::Rising);
        pin.enable_interrupt(exti);
        pin
    }


pub fn setup(
    mut core: stm32h7xx_hal::stm32::CorePeripherals,
    device: stm32h7xx_hal::stm32::Peripherals,
    clock: SystemTimer,
) -> (BoardDevices, hal::device::EXTI, ExtIntPins){
    // Set up RTT logging
    {
        // Enable debug during WFE/WFI-induced sleep
        device.DBGMCU.cr.modify(|_, w| w.dbgsleep_d1().set_bit());

        // Set up RTT channel to use for `rprintln!()` as "best effort".
        // This removes a critical section around the logging and thus allows
        // high-prio tasks to always interrupt at low latency.
        // It comes at a cost:
        // If a high-priority tasks preempts while we are logging something,
        // and if we then also want to log from within that high-preiority task,
        // the high-prio log message will be lost.

        let channels = rtt_target::rtt_init_default!();
        //static mut input : rtt_target::DownChannel  = channels.down.0;
        // Note(unsafe): The closure we pass does not establish a critical section
        // as demanded but it does ensure synchronization and implements a lock.
        unsafe {
            rtt_target::set_print_channel_cs(
                channels.up.0,
                &((|arg, f| {
                    static LOCKED: AtomicBool = AtomicBool::new(false);
                    if LOCKED.compare_exchange_weak(
                        false,
                        true,
                        Ordering::Acquire,
                        Ordering::Relaxed,
                    ) == Ok(false)
                    {
                        f(arg);
                        LOCKED.store(false, Ordering::Release);
                    }
                }) as rtt_target::CriticalSectionFunc),
            );
        }

        static LOGGER: rtt_logger::RTTLogger =
            rtt_logger::RTTLogger::new(log::LevelFilter::Info);
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Trace))
            .unwrap();
        log::info!("Starting");
    }

    let pwr = device.PWR.constrain();
    let vos = pwr.freeze();

    // Enable SRAM3 for the ethernet descriptor ring.
    device.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    // Clear reset flags.
    device.RCC.rsr.write(|w| w.rmvf().set_bit());

    // Select the PLLs for SPI.
    device
        .RCC
        .d2ccip1r
        .modify(|_, w| w.spi123sel().pll2_p().spi45sel().pll2_q());

    device.RCC.d1ccipr.modify(|_, w| w.qspisel().rcc_hclk3());

    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .use_hse(8.mhz())
        .sysclk(design_parameters::SYSCLK)
        .hclk(200.mhz())
        .per_ck(design_parameters::TIMER_FREQUENCY)
        .pll2_p_ck(100.mhz())
        .pll2_q_ck(100.mhz())
        .freeze(vos, &device.SYSCFG);

    let mut syscfg = device.SYSCFG;
    let mut exti = device.EXTI;

    // Before being able to call any code in ITCM, load that code from flash.
    load_itcm();

    let systick = Systick::new(core.SYST, ccdr.clocks.sysclk().0);

    // After ITCM loading.
    core.SCB.enable_icache();

    // log::info!("Hz: {}", asm_delay::bitrate::Hertz(
    //     ccdr.clocks.c_ck().0,
    // ).0);
    let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
        ccdr.clocks.c_ck().0,
    ));

    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = device.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);

    let interrupts_inputs = {
        let int0 = gpioc.pc3.into_pull_up_input();
        let int1 = gpiod.pd1.into_pull_up_input();
        let int2 = gpiog.pg4.into_pull_up_input();
        let int3 = gpiog.pg5.into_pull_up_input();
        let int4 = gpiog.pg6.into_pull_up_input();
        let int5 = gpiog.pg7.into_pull_up_input();
        let int6 = gpioe.pe11.into_pull_up_input();
        let int7 = gpioe.pe12.into_pull_up_input();

        (create_interrupt_conf(int0, &mut exti, &mut syscfg),
         create_interrupt_conf(int1, &mut exti, &mut syscfg),
         create_interrupt_conf(int2, &mut exti, &mut syscfg),
         create_interrupt_conf(int3, &mut exti, &mut syscfg),
         create_interrupt_conf(int4, &mut exti, &mut syscfg),
         create_interrupt_conf(int5, &mut exti, &mut syscfg),
         create_interrupt_conf(int6, &mut exti, &mut syscfg),
         create_interrupt_conf(int7, &mut exti, &mut syscfg),
        )
    };

    let ecp5interface = {//, devicesio) = {  // TODO change to final IO pins here and in type declaration
        let di00 = gpiod.pd7.into_pull_down_input();
        //let mut di01 = gpiog.pg9.into_pull_up_input();
        let di10 = gpiog.pg10.into_pull_down_input();
        //let di11 = gpiog.pg12.into_floating_input();
        let di20 = gpioc.pc2.into_pull_down_input();
        //let di21 = gpioc.pc3.into_floating_input();
        let di30 = gpiob.pb4.into_pull_down_input();
        //let di31 = gpiod.pd3.into_floating_input();
        let di40 = gpiob.pb2.into_pull_down_input();
        //let di41 = gpiob.pb4.into_floating_input();
        let di50 = gpioc.pc10.into_pull_down_input();
        //let di51 = gpioa.pa4.into_floating_input();
        let di60 = gpioe.pe2.into_pull_down_input();
        //let di61 = gpioe.pe4.into_floating_input();
        let di70 = gpioe.pe5.into_pull_down_input();
        //let di71 = gpioe.pe6.into_floating_input();
//
//        di01.make_interrupt_source(&mut syscfg);
//        di01.trigger_on_edge(&mut exti, Edge::Falling);
//        di01.enable_interrupt(&mut exti);

        //(
            (di00, di10, di20, di30, di40, di50, di60, di70)//,
        //    (di01, di11, di21, di31, di41, di51, di61, di71)
        //)
    };

//    let (red, yellow, green, red_in, yellow_in, green_in) = {
//        let nucleo_red = gpiob.pb14.into_push_pull_output();
//        let nucleo_yellow = gpioe.pe1.into_push_pull_output();
//        let nucleo_green = gpiob.pb0.into_push_pull_output();
//        let red_in = gpioc.pc8.into_floating_input();
//        let yellow_in = gpioc.pc9.into_floating_input();
//        let green_in = gpioc.pc12.into_floating_input();
//
//        (nucleo_red, nucleo_yellow, nucleo_green, red_in, yellow_in, green_in)
//    };

    // Check if STM SYS BOARD is in system slot.
    let sysen = gpiob.pb15.into_floating_input();
    if sysen.is_high().unwrap(){
        panic!("STM SYS BOARD on peripheral slot!");
    }

    let mon_bus = {
        let sda = gpiof.pf0.into_alternate_af4().set_open_drain();
        let scl = gpiof.pf1.into_alternate_af4().set_open_drain();
        let i2c = device.I2C2.i2c(
            (scl, sda),
            100.khz(),
            ccdr.peripheral.I2C2,
            &ccdr.clocks,
        );
        let pres = (gpiob.pb14.into_pull_up_input(), gpioc.pc8.into_pull_up_input());
        let mut p_reset = gpioc.pc13.into_push_pull_output();
        p_reset.set_high().unwrap();
        let mut f_reset = gpioc.pc12.into_push_pull_output();
        f_reset.set_high().unwrap();
        let p_ios = (gpiob.pb6.into_floating_input(), gpiob.pb9.into_floating_input(), gpiob.pb10.into_floating_input());
        let f_ios = (gpioc.pc14.into_floating_input(), gpioc.pc15.into_floating_input());

        MonBus{
            i2c,
            p_pres: pres,
            p_rst: p_reset,
            f_rst: f_reset,
            p_ios,
            f_ios,
        }

    };

    // TODO enable to test system board,
    let prst = gpiob.pb0.into_pull_up_input();
    create_interrupt_conf(prst, &mut exti, &mut syscfg);
    let mut rst_per = gpioc.pc0.into_push_pull_output();
    rst_per.set_high().unwrap();
    let _pwrbtn = gpiob.pb1.into_pull_up_input();
    let _pwrfail = gpiob.pb5.into_pull_up_input();

    let mut therm_i2c = {
        let sda = gpiob.pb7.into_alternate_af4().set_open_drain();
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        device.I2C1.i2c(
            (scl, sda),
            100.khz(),
            ccdr.peripheral.I2C1,
            &ccdr.clocks,
        )
    };

    let mut cpcis_i2c = {
        let sda = gpiod.pd13.into_alternate_af4().set_open_drain();
        let scl = gpiod.pd12.into_alternate_af4().set_open_drain();
        device.I2C4.i2c(
            (scl, sda),
            100.khz(),
            ccdr.peripheral.I2C4,
            &ccdr.clocks,
        )
    };

    let mut servmod = (
        gpiof.pf2.into_push_pull_output(),
        gpiof.pf3.into_push_pull_output(),
        gpiof.pf4.into_push_pull_output(),
        gpiof.pf5.into_push_pull_output(),
        gpiof.pf12.into_push_pull_output(),
        gpiof.pf13.into_push_pull_output(),
        gpiof.pf14.into_push_pull_output(),
        gpiof.pf15.into_push_pull_output(),
    );
    servmod.0.set_high().unwrap();
    servmod.1.set_high().unwrap();
    servmod.2.set_high().unwrap();
    servmod.3.set_high().unwrap();
    servmod.4.set_high().unwrap();
    servmod.5.set_high().unwrap();
    servmod.6.set_high().unwrap();
    servmod.7.set_high().unwrap();

   // let mac_addr = smoltcp::wire::EthernetAddress([0x00, 0x0b, 0x00, 0x00, 0x00, 0x00]);
   let mac_addr = smoltcp::wire::EthernetAddress(eeprom::read_eui48(
       &mut therm_i2c,
       &mut delay,
   ));
    log::info!("EUI48: {}", mac_addr);

    let ecp5 = {
        let qspi_interface = {
            let qspi_pins = {   //STM SYS BOARD QSPI pinout
                let _qspi_ncs = gpioc.pc11
                    .into_alternate_af9()  // QUADSPI_BK2_NCS
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let qspi_clk = gpiof
                    .pf10.into_alternate_af9()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let qspi_io0 = gpioe.pe7
                    .into_alternate_af10()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let qspi_io1 = gpioe.pe8
                    .into_alternate_af10()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let qspi_io2 = gpioe.pe9
                    .into_alternate_af10()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let qspi_io3 = gpioe.pe10
                    .into_alternate_af10()
                    .set_speed(hal::gpio::Speed::VeryHigh);

                (qspi_clk, qspi_io0, qspi_io1, qspi_io2, qspi_io3)
            };


            let config = hal::xspi::Config::new(MegaHertz(25))
                .mode(hal::xspi::QspiMode::OneBit)
                .dummy_cycles(5)
                .sampling_edge(hal::xspi::SamplingEdge::Falling)
                .fifo_threshold(1);

            device.QUADSPI.bank2(
                qspi_pins,
                config,
                &ccdr.clocks,
                ccdr.peripheral.QSPI,
            )
        };
        ECP5::new(qspi_interface, ecp5interface)
    };

    let network_devices = {
        let ethernet_pins = {
            // Reset the PHY before configuring pins.
            let mut eth_phy_nrst = gpioe.pe3.into_push_pull_output();
            eth_phy_nrst.set_low().unwrap();
            delay.delay_us(200u8);
            eth_phy_nrst.set_high().unwrap();

            let rmii_ref_clk = gpioa
                .pa1
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_mdio = gpioa
                .pa2
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_mdc = gpioc
                .pc1
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_crs_dv = gpioa
                .pa7
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_rxd0 = gpioc
                .pc4
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_rxd1 = gpioc
                .pc5
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_tx_en = gpiog
                .pg11
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_txd0 = gpiog
                .pg13
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let rmii_txd1 = gpiob
                .pb13
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);

            (
                rmii_ref_clk,
                rmii_mdio,
                rmii_mdc,
                rmii_crs_dv,
                rmii_rxd0,
                rmii_rxd1,
                rmii_tx_en,
                rmii_txd0,
                rmii_txd1,
            )
        };

        // Configure the ethernet controller
        let (eth_dma, eth_mac) = ethernet::new(
            device.ETHERNET_MAC,
            device.ETHERNET_MTL,
            device.ETHERNET_DMA,
            ethernet_pins,
            // Note(unsafe): We only call this function once to take ownership of the
            // descriptor ring.
            unsafe { &mut DES_RING },
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        );

        // Reset and initialize the ethernet phy.
        let mut lan8742a =
            ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();

        unsafe { ethernet::enable_interrupt() };

        // Configure IP address according to DHCP socket availability
        let ip_addrs: smoltcp::wire::IpAddress = option_env!("STATIC_IP")
            .unwrap_or("0.0.0.0")
            .parse()
            .unwrap();

        // Note(unwrap): The hardware configuration function is only allowed to be called once.
        // Unwrapping is intended to panic if called again to prevent re-use of global memory.
        let store =
            cortex_m::singleton!(: NetStorage = NetStorage::default()).unwrap();

        store.ip_addrs[0] = smoltcp::wire::IpCidr::new(ip_addrs, 24);

        let mut routes =
            smoltcp::iface::Routes::new(&mut store.routes_cache[..]);

        routes
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .unwrap();

        let neighbor_cache =
            smoltcp::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

        let mut interface = smoltcp::iface::InterfaceBuilder::new(
            eth_dma,
            &mut store.sockets[..],
        )
        .hardware_addr(smoltcp::wire::HardwareAddress::Ethernet(mac_addr))
        .neighbor_cache(neighbor_cache)
        .ip_addrs(&mut store.ip_addrs[..])
        .routes(routes)
        .finalize();

        if ip_addrs.is_unspecified() {
            interface.add_socket(smoltcp::socket::Dhcpv4Socket::new());
        }

        for storage in store.tcp_socket_storage[..].iter_mut() {
            let tcp_socket = {
                let rx_buffer = smoltcp::socket::TcpSocketBuffer::new(
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::TcpSocketBuffer::new(
                    &mut storage.tx_storage[..],
                );

                smoltcp::socket::TcpSocket::new(rx_buffer, tx_buffer)
            };

            interface.add_socket(tcp_socket);
        }

        for storage in store.udp_socket_storage[..].iter_mut() {
            let udp_socket = {
                let rx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                    &mut storage.rx_metadata[..],
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                    &mut storage.tx_metadata[..],
                    &mut storage.tx_storage[..],
                );

                smoltcp::socket::UdpSocket::new(rx_buffer, tx_buffer)
            };

            interface.add_socket(udp_socket);
        }

        let random_seed = {
            let mut rng =
                device.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
            let mut data = [0u8; 4];
            rng.fill(&mut data).unwrap();
            data
        };

        let mut stack = smoltcp_nal::NetworkStack::new(interface, clock);

        stack.seed_random_port(&random_seed);

        NetworkDevices {
            stack,
            phy: lan8742a,
            mac_address: mac_addr,
        }
    };

//    let mut fp_led_0 = gpiod.pd5.into_push_pull_output();
//    let mut fp_led_1 = gpiod.pd6.into_push_pull_output();
//    let mut fp_led_2 = gpiog.pg4.into_push_pull_output();
//    let mut fp_led_3 = gpiod.pd12.into_push_pull_output();
//
//    fp_led_0.set_low().unwrap();
//    fp_led_1.set_low().unwrap();
//    fp_led_2.set_low().unwrap();
//    fp_led_3.set_low().unwrap();


    let stm_sys_board = BoardDevices {
        systick,
        net: network_devices,
       // devicesio,
        ecp5,
        mon_bus,
        therm_i2c,
        cpcis_i2c,
        servmod,
    };

    // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
    // info!("Built on {}", build_info::BUILT_TIME_UTC);
    // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);
    log::info!("setup() complete");

    (stm_sys_board, exti, interrupts_inputs)
}
