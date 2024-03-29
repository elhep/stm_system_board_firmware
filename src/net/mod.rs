pub use heapless;
pub use miniconf;
pub use serde;

pub mod network_processor;
pub mod telemetry;
pub mod settings;

use crate::hardware::{EthernetPhy, NetworkManager, NetworkStack, SystemTimer};
use network_processor::NetworkProcessor;
use minimq::embedded_nal::IpAddr;
use telemetry::TelemetryClient;
use core::fmt::Write;
use heapless::String;
use miniconf::Miniconf;

pub type NetworkReference =
    smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

/// The default MQTT broker IP address if unspecified.
// TODO change value
pub const DEFAULT_MQTT_BROKER: [u8; 4] = [192, 168, 95, 145];

#[derive(Copy, Clone, PartialEq)]
pub enum UpdateState {
    NoChange,
    Updated,
}

#[derive(Copy, Clone, PartialEq)]
pub enum NetworkState {
    SettingsChanged,
    Updated,
    NoChange,
}

pub struct NetworkUsers<S: Clone + Miniconf>{
    pub miniconf: miniconf::MqttClient<S, NetworkReference, SystemTimer, 512>,
    pub processor: NetworkProcessor,
    pub telemetry: TelemetryClient,
}

impl<S> NetworkUsers <S>
where
    S: Miniconf + Clone,
{
    ///
    /// # Args
    /// * `stack` - The network stack that will be used to share with all network users.
    /// * `phy` - The ethernet PHY connecting the network.
    /// * `clock` - A `SystemTimer` implementing `Clock`.
    /// * `app` - The name of the application.
    /// * `mac` - The MAC address of the network.
    /// * `broker` - The IP address of the MQTT broker to use.
    ///
    /// # Returns
    /// A new struct of network users.
    pub fn new(
        stack: NetworkStack,
        phy: EthernetPhy,
        clock: SystemTimer,
        app: &str,
        mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
        broker: IpAddr,
        settings: S,
    ) -> Self {
        let stack_manager =
            cortex_m::singleton!(: NetworkManager = NetworkManager::new(stack))
                .unwrap();

        let processor =
            NetworkProcessor::new(stack_manager.acquire_stack(), phy);

        let prefix = get_device_prefix(app, mac);

        let miniconf = miniconf::MqttClient::new(
            stack_manager.acquire_stack(),
            &get_client_id(app, "settings", mac),
            &prefix,
            broker,
            clock,
            settings,
        )
        .unwrap();

        let telemetry = TelemetryClient::new(
            stack_manager.acquire_stack(),
            clock,
            &get_client_id(app, "tlm", mac),
            &prefix,
            broker,
        );

        NetworkUsers {
            miniconf,
            processor,
            telemetry,
        }
    }


    /// Update and process all of the network users state.
    ///
    /// # Returns
    /// An indication if any of the network users indicated a state change.
    pub fn update(&mut self) -> NetworkState {
        // Update the MQTT clients.
        self.telemetry.update();

        // Poll for incoming data.
        let poll_result = match self.processor.update() {
            UpdateState::NoChange => NetworkState::NoChange,
            UpdateState::Updated => NetworkState::Updated,
        };

        match self.miniconf.update() {
            Ok(true) => NetworkState::SettingsChanged,
            _ => poll_result,
        }
    }
}

/// Get an MQTT client ID for a client.
///
/// # Args
/// * `app` - The name of the application
/// * `client` - The unique tag of the client
/// * `mac` - The MAC address of the device.
///
/// # Returns
/// A client ID that may be used for MQTT client identification.
fn get_client_id(
    app: &str,
    client: &str,
    mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
) -> String<64> {
    let mut identifier = String::new();
    write!(&mut identifier, "{}-{}-{}", app, mac, client).unwrap();
    identifier
}

/// Get the MQTT prefix of a device.
///
/// # Args
/// * `app` - The name of the application that is executing.
/// * `mac` - The ethernet MAC address of the device.
///
/// # Returns
/// The MQTT prefix used for this device.
pub fn get_device_prefix(
    app: &str,
    mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
) -> String<128> {
    // Note(unwrap): The mac address + binary name must be short enough to fit into this string. If
    // they are defined too long, this will panic and the device will fail to boot.
    let mut prefix: String<128> = String::new();
    write!(&mut prefix, "dt/sinara/{}/{}", app, mac).unwrap();

    prefix
}