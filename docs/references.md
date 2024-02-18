https://www.geeksforgeeks.org/how-to-put-wifi-interface-into-monitor-mode-in-linux/
http://wifinigel.blogspot.com/2013/11/what-are-radiotap-headers.html
https://www.kernel.org/doc/html/v6.3/networking/radiotap-headers.html
https://howiwifi.com/2020/07/13/802-11-frame-types-and-formats/
https://en.wikipedia.org/wiki/802.11_frame_types#Types_and_subtypes
https://www.radiotap.org/
https://askubuntu.com/questions/1330483/wifi-interface-mode-switches-automatically-back-to-managed-ubuntu-20-04
https://nxnjz.net/2018/06/how-to-increase-the-transmission-power-of-a-wifi-card-adapter-txpower/
https://ubuntuforums.org/showthread.php?t=2002796

https://www.hackster.io/p99will/esp32-wifi-mac-scanner-sniffer-promiscuous-4c12f4
https://stackoverflow.com/questions/18838451/difference-between-socket-pf-packet-and-socket-af-inet-in-python

https://users.rust-lang.org/t/weird-behaviour-reading-from-a-linux-device-event-stream/43816/9

// const WIFI_PACKET: [u8; 41] = [
//     0x00, 0x00, // <-- radiotap version
//     0x0f, 0x00, // <- radiotap header length
//     0x04, 0x0c, 0x00, 0x00, // <-- bitmap
//     0x6c, // <-- rate
//     0x09, 0xA8, 0x80, 0x00, //<-- Channel
//     0x0c, //<-- tx power
//     0x01, //  antenna
//     0x08, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x13, 0x22, 0x33, 0x44, 0x55, 0x66,
//     0x13, 0x22, 0x33, 0x44, 0x55, 0x66, 0x10, 0x86, 0xF1, 0xF1,
// ];

    // let radiotap_header_payload = [
    //     RATE_1_MBPS,
    //     0x00, /*Wft is this byte? maybe padding?*/
    //     0xA8,
    //     0x09, // <- channel freq 2472
    //     0x80,
    //     0x00, // <- channel flags 2ghz
    //     0x0c, // <- Tx power
    // ];

    // let data = [
    //     0x08, //Frame type
    //     0x00, //Flags
    //     0x00, 0x00, //Duration
    //     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // Receiver Address
    //     0x13, 0x22, 0x33, 0x44, 0x55, 0x66, // Transmitter Address
    //     0x13, 0x22, 0x33, 0x44, 0x55, 0x66, // Destination Address
    //     0x00, 0x01, //Sequence Control
    //     0x13, 0x22, 0x33, 0x44, 0x55, 0x66, // Source Address
    // ];

    // let data = [0x01];

    // let wifi_frame= WifiPacketPayload{

    // }
    // let data_array = unsafe {
    //     core::slice::from_raw_parts_mut(
    //         &wifi_frame as *const _ as *mut u8,
    //         mem::size_of::<NoDsWifiPacketFrame<4>>(),
    //     )
    // };

    // println!("Packet Array{:02X?}", packet_array);

    // let radiotap_packet = Radiotap::from_bytes(&packet_array).unwrap();
    // // match radiotap_header.antenna. {

    // // }
    // println!("Radiotap Freq {}", radiotap_packet.channel.unwrap().freq);
    // println!(
    //     "Radiotap Rate {:02X?}",
    //     radiotap_header.tx_power.unwrap().value
    // );

    // radiotap::Radiotap::from()

    // match libwifi::parse_frame(&data) {
    //     Ok(frame) => {
    //         println!("Got frame: {:?}", frame);
    //     }
    //     Err(err) => {
    //         println!("Error during parsing :\n{}", err);
    //     }
    // };