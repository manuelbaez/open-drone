use opencv::{
    core::{Mat, Vector},
    imgcodecs,
    prelude::*,
    videoio,
};
use std::{thread::sleep, time::Duration};

use std::fs;
use std::io::Write;
use std::net::TcpListener;

fn main() {
    let mut cam =
        videoio::VideoCapture::new(0, videoio::CAP_ANY).expect("Failed to get video capture");
    let mut frame = Mat::default();
    let mut buf = Vector::new();
    // let listener = TcpListener::bind("92.168.15.30:8080").unwrap();
    loop {
        let mut file = fs::OpenOptions::new()
            .create(true) // To create a new file
            .write(true)
            // either use the ? operator or unwrap since it returns a Result
            .open("./image.jpg")
            .unwrap();
        cam.read(&mut frame).expect("Failed to capture frame");
        buf.clear();
        let _ = imgcodecs::imencode(".jpg", &frame, &mut buf, &Vector::new());

        let image_data = format!(
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
            buf.len()
        );

        file.write_all(buf.as_slice());
        // println!("{:?} {:?}", image_data.as_bytes(), buf);
        sleep(Duration::from_millis(200));
    }
}
