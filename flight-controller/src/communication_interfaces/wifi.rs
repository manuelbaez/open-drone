use esp_idf_svc::sys::{
    esp_wifi_init, esp_wifi_set_channel, esp_wifi_set_mode, esp_wifi_set_promiscuous,
    esp_wifi_set_promiscuous_filter, esp_wifi_set_promiscuous_rx_cb, esp_wifi_set_storage,
    esp_wifi_start, g_wifi_default_wpa_crypto_funcs, g_wifi_feature_caps, g_wifi_osi_funcs,
    nvs_flash_init, wifi_init_config_t, wifi_mode_t_WIFI_MODE_NULL, wifi_osi_funcs_t,
    wifi_promiscuous_filter_t, wifi_promiscuous_pkt_t, wifi_second_chan_t_WIFI_SECOND_CHAN_NONE,
    wifi_storage_t_WIFI_STORAGE_RAM, CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM,
    CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM, CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM,
    CONFIG_ESP_WIFI_TX_BUFFER_TYPE, WIFI_AMPDU_RX_ENABLED, WIFI_AMPDU_TX_ENABLED,
    WIFI_AMSDU_TX_ENABLED, WIFI_CACHE_TX_BUFFER_NUM, WIFI_CSI_ENABLED, WIFI_DEFAULT_RX_BA_WIN,
    WIFI_DYNAMIC_TX_BUFFER_NUM, WIFI_INIT_CONFIG_MAGIC, WIFI_MGMT_SBUF_NUM,
    WIFI_NANO_FORMAT_ENABLED, WIFI_NVS_ENABLED, WIFI_PROMIS_FILTER_MASK_DATA,
    WIFI_PROMIS_FILTER_MASK_MGMT, WIFI_SOFTAP_BEACON_MAX_LEN, WIFI_STATIC_TX_BUFFER_NUM,
    WIFI_STA_DISCONNECTED_PM_ENABLED, WIFI_TASK_CORE_ID,
};
use std::ffi::c_void;

#[repr(C, packed)]
struct MacAddr {
    mac: [u8; 6],
}
#[repr(C, packed)]
struct WifiPacketPayload {
    fctl: u16,
    duration: u16,
    da: MacAddr,
    sa: MacAddr,
    bssid: MacAddr,
    seqctl: u16,
    addr_4: MacAddr,
    payload: &'static [u8],
}

pub struct WifiSniffer;

impl WifiSniffer {
    unsafe extern "C" fn sniffer(buffer: *mut c_void, _handler: u32) {
        let packet_pointer: *mut wifi_promiscuous_pkt_t = buffer as *mut wifi_promiscuous_pkt_t;
        let packet = packet_pointer.read();
        let length = packet.rx_ctrl.sig_len();
        let wifi_packet_payload_pointer = &packet.payload as *const _ as *const WifiPacketPayload;
        let wifi_packet_payload = wifi_packet_payload_pointer.read();
        // wifi_hdr

        let payload = wifi_packet_payload.addr_4;

        log::info!("Packet {} -- {:?}", length, payload.mac);
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

    pub fn init_promiscous(channel: u8) {
        unsafe {
            let filter: wifi_promiscuous_filter_t = wifi_promiscuous_filter_t {
                filter_mask: WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
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
            esp_wifi_set_channel(channel, wifi_second_chan_t_WIFI_SECOND_CHAN_NONE);
        }
    }
}
