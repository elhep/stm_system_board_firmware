use heapless::{String, Vec};
use minimq::{QoS, Retain};
use serde::Serialize;

use super::NetworkReference;
use crate::hardware::SystemTimer;
use minimq::embedded_nal::IpAddr;
use core::fmt::Write;

/// The telemetry client for reporting telemetry data over MQTT.
pub struct TelemetryClient {
    mqtt: minimq::Minimq<NetworkReference, SystemTimer, 512, 1>,
    telemetry_topic: String<128>,
 //   _telemetry: core::marker::PhantomData<T>,
}

impl TelemetryClient {
    /// Construct a new telemetry client.
    ///
    /// # Args
    /// * `stack` - A reference to the (shared) underlying network stack.
    /// * `clock` - A `SystemTimer` implementing `Clock`.
    /// * `client_id` - The MQTT client ID of the telemetry client.
    /// * `prefix` - The device prefix to use for MQTT telemetry reporting.
    /// * `broker` - The IP address of the MQTT broker to use.
    ///
    /// # Returns
    /// A new telemetry client.
    pub fn new(
        stack: NetworkReference,
        clock: SystemTimer,
        client_id: &str,
        prefix: &str,
        broker: IpAddr,
    ) -> Self {
        let mqtt =
            minimq::Minimq::new(broker, client_id, stack, clock).unwrap();

        let mut telemetry_topic: String<128> = String::from(prefix);
        telemetry_topic.push_str("/telemetry").unwrap();

        Self {
            mqtt,
            telemetry_topic,
        }
    }

    /// Publish telemetry over MQTT
    ///
    /// # Note
    /// Telemetry is reported in a "best-effort" fashion. Failure to transmit telemetry will cause
    /// it to be silently dropped.
    ///
    /// # Args
    /// * `telemetry` - The telemetry to report
    pub fn publish<T: Serialize>(&mut self, device_name: &str ,telemetry: &T) {
        let telemetry: Vec<u8, 512> =
            serde_json_core::to_vec(telemetry).unwrap();
        self.mqtt
            .client
            .publish(
                &get_tlm_topic(&self.telemetry_topic, device_name),
                &telemetry,
                QoS::AtMostOnce,
                Retain::NotRetained,
                &[],
            )
            .ok();
    }

    /// Update the telemetry client
    ///
    /// # Note
    /// This function is provided to force the underlying MQTT state machine to process incoming
    /// and outgoing messages. Without this, the client will never connect to the broker. This
    /// should be called regularly.
    pub fn update(&mut self) {      //TODO check if can be delated
        match self.mqtt.poll(|_client, _topic, _message, _properties| {}) {
            Err(minimq::Error::Network(
                smoltcp_nal::NetworkError::NoIpAddress,
            )) => {}

            Err(error) => log::info!("Unexpected error: {:?}", error),
            _ => {}
        }
    }
}

pub fn get_tlm_topic(
    topic: &str,
    device: &str,
) -> String<128> {
    // Note(unwrap): The mac address + binary name must be short enough to fit into this string. If
    // they are defined too long, this will panic and the device will fail to boot.
    let mut full_topic: String<128> = String::new();
    write!(&mut full_topic, "{}/{}", topic, device).unwrap();

    full_topic
}
