use esp_idf_svc::{
    hal::delay::FreeRtos,
    sys::{
        esp_wifi_80211_tx, esp_wifi_init, esp_wifi_internal_tx, esp_wifi_set_channel,
        esp_wifi_set_max_tx_power, esp_wifi_set_mode, esp_wifi_set_promiscuous,
        esp_wifi_set_promiscuous_filter, esp_wifi_set_promiscuous_rx_cb, esp_wifi_set_storage,
        esp_wifi_start, g_wifi_default_wpa_crypto_funcs, g_wifi_feature_caps, g_wifi_osi_funcs,
        nvs_flash_init, wifi_init_config_t, wifi_interface_t_WIFI_IF_NAN,
        wifi_mode_t_WIFI_MODE_NULL, wifi_osi_funcs_t, wifi_pkt_rx_ctrl_t,
        wifi_promiscuous_filter_t, wifi_promiscuous_pkt_t, wifi_promiscuous_pkt_type_t,
        wifi_promiscuous_pkt_type_t_WIFI_PKT_CTRL, wifi_promiscuous_pkt_type_t_WIFI_PKT_DATA,
        wifi_promiscuous_pkt_type_t_WIFI_PKT_MGMT, wifi_second_chan_t_WIFI_SECOND_CHAN_NONE,
        wifi_storage_t_WIFI_STORAGE_RAM, CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM,
        CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM, CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM,
        CONFIG_ESP_WIFI_TX_BUFFER_TYPE, WIFI_AMPDU_RX_ENABLED, WIFI_AMPDU_TX_ENABLED,
        WIFI_AMSDU_TX_ENABLED, WIFI_CACHE_TX_BUFFER_NUM, WIFI_CSI_ENABLED, WIFI_DEFAULT_RX_BA_WIN,
        WIFI_DYNAMIC_TX_BUFFER_NUM, WIFI_INIT_CONFIG_MAGIC, WIFI_MGMT_SBUF_NUM,
        WIFI_NANO_FORMAT_ENABLED, WIFI_NVS_ENABLED, WIFI_PROMIS_FILTER_MASK_ALL,
        WIFI_PROMIS_FILTER_MASK_DATA, WIFI_PROMIS_FILTER_MASK_MGMT, WIFI_SOFTAP_BEACON_MAX_LEN,
        WIFI_STATIC_TX_BUFFER_NUM, WIFI_STA_DISCONNECTED_PM_ENABLED, WIFI_TASK_CORE_ID,
    },
};
use std::{
    ffi::c_void,
    mem,
    ops::Deref,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex, RwLock,
    },
};
use wifi_protocol::{
    ieee80211_frames::{GenericWifiPacketFrameHeader, IBSSWifiPacketFrame},
    payloads::{CustomSAPs, DroneMovementsFramePayload},
};

use crate::config::constants::{PAIRING_BSSID_ADDRESS, TRANSMITTER_ADDRESS};

pub type ControllerInput = DroneMovementsFramePayload;

//Had to construct my own struct as I couldn't work with the __IncompleteArrayField<> in wifi_promiscuous_pkt_t
// #[derive(Debug, Default)]
#[repr(C)]
pub struct WifiPromiscousPacket {
    pub rx_ctrl: wifi_pkt_rx_ctrl_t,
    pub payload: GenericWifiPacketFrameHeader,
}

pub static CONTROLLER_INPUT_DATA: RwLock<ControllerInput> = RwLock::new(ControllerInput {
    yaw: 0,
    pitch: 0,
    roll: 0,
    throttle: 0,
    kill_motors: false,
    start: false,
    calibrate: false,
});
pub static DATA_READY_LOCK: AtomicBool = AtomicBool::new(false);

pub struct WifiController;

impl WifiController {
    fn handle_controller_input(input: ControllerInput) {
        let mut input_data_lock = CONTROLLER_INPUT_DATA.write().unwrap();

        input_data_lock.kill_motors = input.kill_motors;
        input_data_lock.start = input.start;
        input_data_lock.roll = input.roll;
        input_data_lock.pitch = input.pitch;
        input_data_lock.yaw = input.yaw;
        input_data_lock.throttle = input.throttle;
        input_data_lock.calibrate = input.calibrate;
        DATA_READY_LOCK.store(true, Ordering::Release);
        drop(input_data_lock);
    }

    unsafe extern "C" fn sniffer(buffer: *mut c_void, _packet_type: wifi_promiscuous_pkt_type_t) {
        // if packet_type == wifi_promiscuous_pkt_type_t_WIFI_PKT_DATA {
        let packet_pointer = buffer as *const WifiPromiscousPacket;
        let esp_packet = packet_pointer.read();
        let wifi_frame_length = esp_packet.rx_ctrl.sig_len();

        let data_ptr = buffer as *const u8;
        let rx_ctrl_length = mem::size_of::<wifi_pkt_rx_ctrl_t>();
        let total_length = rx_ctrl_length + wifi_frame_length as usize;
        let total_struct_data = core::slice::from_raw_parts(data_ptr, total_length);
        let wifi_frame_data = total_struct_data.split_at(rx_ctrl_length).1;

        let wifi_frame = esp_packet.payload;
        let bssid = wifi_frame.address_3;
        let transmitter_addr = wifi_frame.address_2;

        if bssid.mac == PAIRING_BSSID_ADDRESS && transmitter_addr.mac == TRANSMITTER_ADDRESS {
            let llc = wifi_frame.logical_link_control;
            let dsap = llc.dsap();
            match CustomSAPs::try_from(dsap).unwrap() {
                CustomSAPs::ControllerFrame => {
                    let parsed_packet: IBSSWifiPacketFrame<DroneMovementsFramePayload> =
                        mem::zeroed();

                    let parsed_packet_size =
                        mem::size_of::<IBSSWifiPacketFrame<DroneMovementsFramePayload>>();
                    let parsed_packet_buffer = core::slice::from_raw_parts_mut(
                        &parsed_packet as *const _ as *mut u8,
                        parsed_packet_size,
                    );

                    parsed_packet_buffer.clone_from_slice(&wifi_frame_data[..parsed_packet_size]);

                    let data = parsed_packet.data;
                    Self::handle_controller_input(data);

                    // log::info!("Packet {:02x?}", frame_control);
                }
                _ => (),
            }
        }
    }

    unsafe fn get_wifi_default_config() -> wifi_init_config_t {
        wifi_init_config_t {
            osi_funcs: &g_wifi_osi_funcs as *const _ as *mut wifi_osi_funcs_t,
            wpa_crypto_funcs: g_wifi_default_wpa_crypto_funcs,
            static_rx_buf_num: CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM as i32,
            dynamic_rx_buf_num: CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM as i32,
            tx_buf_type: CONFIG_ESP_WIFI_TX_BUFFER_TYPE as i32,
            static_tx_buf_num: WIFI_STATIC_TX_BUFFER_NUM as i32,
            dynamic_tx_buf_num: WIFI_DYNAMIC_TX_BUFFER_NUM as i32,
            cache_tx_buf_num: WIFI_CACHE_TX_BUFFER_NUM as i32,
            csi_enable: WIFI_CSI_ENABLED as i32,
            ampdu_rx_enable: WIFI_AMPDU_RX_ENABLED as i32,
            ampdu_tx_enable: WIFI_AMPDU_TX_ENABLED as i32,
            amsdu_tx_enable: WIFI_AMSDU_TX_ENABLED as i32,
            nvs_enable: WIFI_NVS_ENABLED as i32,
            nano_enable: WIFI_NANO_FORMAT_ENABLED as i32,
            rx_ba_win: WIFI_DEFAULT_RX_BA_WIN as i32,
            wifi_task_core_id: WIFI_TASK_CORE_ID as i32,
            beacon_max_len: WIFI_SOFTAP_BEACON_MAX_LEN as i32,
            mgmt_sbuf_num: WIFI_MGMT_SBUF_NUM as i32,
            feature_caps: g_wifi_feature_caps,
            sta_disconnected_pm: WIFI_STA_DISCONNECTED_PM_ENABLED != 0,
            espnow_max_encrypt_num: CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM as i32,
            magic: WIFI_INIT_CONFIG_MAGIC as i32,
        }
    }

    pub fn init_monitor(channel: u8, shared_controller_input: Arc<RwLock<ControllerInput>>) {
        unsafe {
            let filter: wifi_promiscuous_filter_t = wifi_promiscuous_filter_t {
                // filter_mask: WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
                filter_mask: WIFI_PROMIS_FILTER_MASK_DATA,
            };
            let filter_pointer = &filter as *const _ as *const wifi_promiscuous_filter_t;

            let cfg = Self::get_wifi_default_config();
            nvs_flash_init();
            esp_wifi_init(&cfg);
            esp_wifi_set_storage(wifi_storage_t_WIFI_STORAGE_RAM);
            esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL);
            esp_wifi_start();
            esp_wifi_set_promiscuous(true);
            esp_wifi_set_promiscuous_filter(filter_pointer);
            esp_wifi_set_promiscuous_rx_cb(Option::Some(Self::sniffer));
            // esp_wifi_set_max_tx_power(80); //Set to 20dbm transmit power
            esp_wifi_set_channel(channel, wifi_second_chan_t_WIFI_SECOND_CHAN_NONE);
            // esp_wifi_80211_tx(wifi_interface_t_WIFI_IF_NAN, buffer, len, en_sys_seq);
            // esp_wifi_internal_tx(wifi_if, buffer, len)
        }

        loop {
            let ready = DATA_READY_LOCK.load(Ordering::Acquire);
            if ready {
                let mut shared_controller_input_lock = shared_controller_input.write().unwrap();
                let intermediate_input_lock = CONTROLLER_INPUT_DATA.read().unwrap();
                shared_controller_input_lock.kill_motors = intermediate_input_lock.kill_motors;
                shared_controller_input_lock.start = intermediate_input_lock.start;
                shared_controller_input_lock.calibrate = intermediate_input_lock.calibrate;
                shared_controller_input_lock.roll = intermediate_input_lock.roll;
                shared_controller_input_lock.pitch = intermediate_input_lock.pitch;
                shared_controller_input_lock.yaw = intermediate_input_lock.yaw;
                shared_controller_input_lock.throttle = intermediate_input_lock.throttle;
                DATA_READY_LOCK.store(false, Ordering::Release);
                drop(shared_controller_input_lock);
                drop(intermediate_input_lock);
                continue;
            }

            FreeRtos::delay_ms(1);
        }
    }
}
