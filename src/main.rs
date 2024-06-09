use std::sync::{Arc, Mutex};
use std::thread;
use std::f64::consts::PI;
use std::time::{Instant, Duration};
use std::collections::HashMap;
use serialport::prelude::*;
use serialport::SerialPort;
use std::io::Read;
use std::sync::atomic::{AtomicBool, Ordering};


struct Point2D {
    degree: f64,     // 0.0 ~ 359.9, 0 指向前方, 顺时针
    distance: u32,   // 距离 mm
    confidence: u32, // 置信度 典型值=200
}

impl Point2D {
    fn new(degree: f64, distance: u32, confidence: u32) -> Self {
        Self {
            degree,
            distance,
            confidence,
        }
    }
}

impl Default for Point2D {
    fn default() -> Self {
        Self {
            degree: 0.0,
            distance: 0,
            confidence: 0,
        }
    }
}

struct RadarPackage {
    rotation_spd: u32,           // 转速 deg/s
    start_degree: f64,           // 扫描开始角度
    points: [Point2D; 12],       // 12个点的数据
    stop_degree: f64,            // 扫描结束角度
    time_stamp: u32,             // 时间戳 ms 记满30000后重置
    recent_update_result: bool,  // 最近更新结果
}

impl RadarPackage {
    fn new(datas: Option<&[u32]>) -> Self {
        let mut package = Self {
            rotation_spd: 0,
            start_degree: 0.0,
            points: Default::default(),
            stop_degree: 0.0,
            time_stamp: 0,
            recent_update_result: false,
        };

        if let Some(data) = datas {
            package.fill_data(data);
        }

        package
    }

    fn fill_data(&mut self, datas: &[u32]) {
        self.rotation_spd = datas[0];
        self.start_degree = datas[1] as f64 * 0.01;
        self.stop_degree = datas[26] as f64 * 0.01;
        self.time_stamp = datas[27];

        let deg_step = (self.stop_degree - self.start_degree) % 360.0 / 11.0;
        for (n, point) in self.points.iter_mut().enumerate() {
            point.distance = datas[2 + n * 2];
            point.confidence = datas[3 + n * 2];
            point.degree = (self.start_degree + n as f64 * deg_step) % 360.0;
        }
    }
}

impl Default for RadarPackage {
    fn default() -> Self {
        Self {
            rotation_spd: 0,
            start_degree: 0.0,
            points: Default::default(),
            stop_degree: 0.0,
            time_stamp: 0,
            recent_update_result: false,
        }
    }
}

struct Point2D {
    degree: f64,
    distance: u32,
    confidence: u32,
}

impl Default for Point2D {
    fn default() -> Self {
        Self {
            degree: 0.0,
            distance: 0,
            confidence: 0,
        }
    }
}

struct RadarPackage {
    rotation_spd: u32,
    start_degree: f64,
    points: [Point2D; 12],
    stop_degree: f64,
    time_stamp: u32,
    recent_update_result: bool,
}

impl RadarPackage {
    fn new(datas: Option<&[u32]>) -> Self {
        let mut package = Self {
            rotation_spd: 0,
            start_degree: 0.0,
            points: [Point2D::default(); 12],
            stop_degree: 0.0,
            time_stamp: 0,
            recent_update_result: false,
        };

        if let Some(data) = datas {
            package.fill_data(data);
        }

        package
    }

    fn fill_data(&mut self, datas: &[u32]) {
        self.rotation_spd = datas[0];
        self.start_degree = datas[1] as f64 * 0.01;
        self.stop_degree = datas[26] as f64 * 0.01;
        self.time_stamp = datas[27];

        let deg_step = (self.stop_degree - self.start_degree) % 360.0 / 11.0;
        for (n, point) in self.points.iter_mut().enumerate() {
            point.distance = datas[2 + n * 2];
            point.confidence = datas[3 + n * 2];
            point.degree = (self.start_degree + n as f64 * deg_step) % 360.0;
        }
    }
}

struct Map360 {
    acc: usize,
    remap: usize,
    mode_min: u8,
    mode_max: u8,
    mode_avg: u8,
    update_mode: u8,
    data: Vec<i64>,
    time_stamp: Vec<f64>,
    confidence_threshold: u32,
    distance_threshold: u32,
    timeout_clear: bool,
    timeout_time: f64,
    rotation_spd: u32,
    update_count: u32,
    rad_arr: Vec<f64>,
    deg_arr: Vec<f64>,
    sin_arr: Vec<f64>,
    cos_arr: Vec<f64>,
    avail_points: usize,
}

impl Map360 {
    fn new() -> Self {
        let acc = 3;
        let size = 360 * acc;
        let rad_arr: Vec<f64> = (0..size).map(|x| (x as f64 / acc as f64).to_radians()).collect();
        let deg_arr: Vec<f64> = (0..size).map(|x| x as f64 / acc as f64).collect();
        let sin_arr: Vec<f64> = rad_arr.iter().map(|&x| x.sin()).collect();
        let cos_arr: Vec<f64> = rad_arr.iter().map(|&x| x.cos()).collect();

        Self {
            acc,
            remap: 2,
            mode_min: 0,
            mode_max: 1,
            mode_avg: 2,
            update_mode: 0,
            data: vec![-1; size],
            time_stamp: vec![0.0; size],
            confidence_threshold: 20,
            distance_threshold: 10,
            timeout_clear: true,
            timeout_time: 1.0,
            rotation_spd: 0,
            update_count: 0,
            rad_arr,
            deg_arr,
            sin_arr,
            cos_arr,
            avail_points: 0,
        }
    }

    fn update(&mut self, data: &RadarPackage) {
        let mut deg_values_dict: std::collections::HashMap<usize, Vec<u32>> = std::collections::HashMap::new();
        let current_time = Instant::now().elapsed().as_secs_f64();

        for point in data.points.iter() {
            if point.distance < self.distance_threshold || point.confidence < self.confidence_threshold {
                continue;
            }

            let base = (point.degree * self.acc as f64).round() as usize;
            if self.remap == 0 {
                let key = base % (360 * self.acc);
                deg_values_dict.entry(key).or_insert_with(Vec::new).push(point.distance);
            } else {
                let range = (base as isize - self.remap as isize)..=(base as isize + self.remap as isize);
                for deg in range {
                    let key = (deg.rem_euclid(360 * self.acc as isize)) as usize;
                    deg_values_dict.entry(key).or_insert_with(Vec::new).push(point.distance);
                }
            }
        }

        for (deg, values) in deg_values_dict.iter() {
            let val = match self.update_mode {
                0 => *values.iter().min().unwrap_or(&-1),
                1 => *values.iter().max().unwrap_or(&-1),
                2 => (values.iter().sum::<u32>() as f64 / values.len() as f64).round() as u32,
                _ => -1,
            };

            self.data[*deg] = val as i64;
            if self.timeout_clear {
                self.time_stamp[*deg] = current_time;
            }
        }

        if self.timeout_clear {
            let timeout_threshold = current_time - self.timeout_time;
            for (i, timestamp) in self.time_stamp.iter_mut().enumerate() {
                if *timestamp < timeout_threshold {
                    self.data[i] = -1;
                }
            }
        }

        self.update_count += 1;
        self.rotation_spd = data.rotation_spd / 360;
        self.avail_points = self.data.iter().filter(|&&d| d != -1).count();
    }

    fn clear(&mut self) {
        self.data.fill(-1);
        self.time_stamp.fill(0.0);
    }

    fn output_cloud(&self, scale: f64, size: usize) -> Vec<Vec<u8>> {
        let mut img = vec![vec![0; size]; size];
        let center = size as f64 / 2.0;
        let points_pos: Vec<(f64, f64)> = self.data.iter().enumerate().map(|(i, &dist)| {
            let dist = dist as f64 * scale;
            (dist * self.sin_arr[i], -dist * self.cos_arr[i])
        }).collect();

        for (i, &(x, y)) in points_pos.iter().enumerate() {
            if self.data[i] != -1 {
                let (x, y) = (x + center, y + center);
                if x >= 0.0 && x < size as f64 && y >= 0.0 && y < size as f64 {
                    img[y as usize][x as usize] = 255;
                }
            }
        }

        img
    }
}

struct Point2D {
    degree: f64,
    distance: u32,
    confidence: u32,
}

impl Default for Point2D {
    fn default() -> Self {
        Self {
            degree: 0.0,
            distance: 0,
            confidence: 0,
        }
    }
}

struct RadarPackage {
    rotation_spd: u32,
    start_degree: f64,
    points: [Point2D; 12],
    stop_degree: f64,
    time_stamp: u32,
    recent_update_result: bool,
}

impl RadarPackage {
    fn new(datas: Option<&[u32]>) -> Self {
        let mut package = Self {
            rotation_spd: 0,
            start_degree: 0.0,
            points: [Point2D::default(); 12],
            stop_degree: 0.0,
            time_stamp: 0,
            recent_update_result: false,
        };

        if let Some(data) = datas {
            package.fill_data(data);
        }

        package
    }

    fn fill_data(&mut self, datas: &[u32]) {
        self.rotation_spd = datas[0];
        self.start_degree = datas[1] as f64 * 0.01;
        self.stop_degree = datas[26] as f64 * 0.01;
        self.time_stamp = datas[27];

        let deg_step = (self.stop_degree - self.start_degree) % 360.0 / 11.0;
        for (n, point) in self.points.iter_mut().enumerate() {
            point.distance = datas[2 + n * 2];
            point.confidence = datas[3 + n * 2];
            point.degree = (self.start_degree + n as f64 * deg_step) % 360.0;
        }
    }
}

struct Map360 {
    acc: usize,
    remap: usize,
    mode_min: u8,
    mode_max: u8,
    mode_avg: u8,
    update_mode: u8,
    data: Vec<i64>,
    time_stamp: Vec<f64>,
    confidence_threshold: u32,
    distance_threshold: u32,
    timeout_clear: bool,
    timeout_time: f64,
    rotation_spd: u32,
    update_count: u32,
    rad_arr: Vec<f64>,
    deg_arr: Vec<f64>,
    sin_arr: Vec<f64>,
    cos_arr: Vec<f64>,
    avail_points: usize,
}

impl Map360 {
    fn new() -> Self {
        let acc = 3;
        let size = 360 * acc;
        let rad_arr: Vec<f64> = (0..size).map(|x| (x as f64 / acc as f64).to_radians()).collect();
        let deg_arr: Vec<f64> = (0..size).map(|x| x as f64 / acc as f64).collect();
        let sin_arr: Vec<f64> = rad_arr.iter().map(|&x| x.sin()).collect();
        let cos_arr: Vec<f64> = rad_arr.iter().map(|&x| x.cos()).collect();

        Self {
            acc,
            remap: 2,
            mode_min: 0,
            mode_max: 1,
            mode_avg: 2,
            update_mode: 0,
            data: vec![-1; size],
            time_stamp: vec![0.0; size],
            confidence_threshold: 20,
            distance_threshold: 10,
            timeout_clear: true,
            timeout_time: 1.0,
            rotation_spd: 0,
            update_count: 0,
            rad_arr,
            deg_arr,
            sin_arr,
            cos_arr,
            avail_points: 0,
        }
    }

    fn update(&mut self, data: &RadarPackage) {
        let mut deg_values_dict: HashMap<usize, Vec<u32>> = HashMap::new();
        let current_time = Instant::now().elapsed().as_secs_f64();

        for point in data.points.iter() {
            if point.distance < self.distance_threshold || point.confidence < self.confidence_threshold {
                continue;
            }

            let base = (point.degree * self.acc as f64).round() as usize;
            if self.remap == 0 {
                let key = base % (360 * self.acc);
                deg_values_dict.entry(key).or_insert_with(Vec::new).push(point.distance);
            } else {
                let range = (base as isize - self.remap as isize)..=(base as isize + self.remap as isize);
                for deg in range {
                    let key = (deg.rem_euclid(360 * self.acc as isize)) as usize;
                    deg_values_dict.entry(key).or_insert_with(Vec::new).push(point.distance);
                }
            }
        }

        for (deg, values) in deg_values_dict.iter() {
            let val = match self.update_mode {
                0 => *values.iter().min().unwrap_or(&-1),
                1 => *values.iter().max().unwrap_or(&-1),
                2 => (values.iter().sum::<u32>() as f64 / values.len() as f64).round() as u32,
                _ => -1,
            };

            self.data[*deg] = val as i64;
            if self.timeout_clear {
                self.time_stamp[*deg] = current_time;
            }
        }

        if self.timeout_clear {
            let timeout_threshold = current_time - self.timeout_time;
            for (i, timestamp) in self.time_stamp.iter_mut().enumerate() {
                if *timestamp < timeout_threshold {
                    self.data[i] = -1;
                }
            }
        }

        self.update_count += 1;
        self.rotation_spd = data.rotation_spd / 360;
        self.avail_points = self.data.iter().filter(|&&d| d != -1).count();
    }

    fn clear(&mut self) {
        self.data.fill(-1);
        self.time_stamp.fill(0.0);
    }

    fn output_cloud(&self, scale: f64, size: usize) -> Vec<Vec<u8>> {
        let mut img = vec![vec![0; size]; size];
        let center = size as f64 / 2.0;
        let points_pos: Vec<(f64, f64)> = self.data.iter().enumerate().map(|(i, &dist)| {
            let dist = dist as f64 * scale;
            (dist * self.sin_arr[i], -dist * self.cos_arr[i])
        }).collect();

        for (i, &(x, y)) in points_pos.iter().enumerate() {
            if self.data[i] != -1 {
                let (x, y) = (x + center, y + center);
                if x >= 0.0 && x < size as f64 && y >= 0.0 && y < size as f64 {
                    img[y as usize][x as usize] = 255;
                }
            }
        }

        img
    }
}

struct RadarSerialUpdater {
    base: Map360,
    serial_running: AtomicBool,
    package: RadarPackage,
    serial_port: Option<Box<dyn SerialPort>>,
    _update_callback: Option<Box<dyn Fn()>>,
    _map_updated_event: Arc<Mutex<()>>,
    radar_unpack_fmt: &'static str,
    thread_list: Vec<thread::JoinHandle<()>>,
}

impl RadarSerialUpdater {
    fn new() -> Self {
        Self {
            base: Map360::new(),
            serial_running: AtomicBool::new(false),
            package: RadarPackage::new(None),
            serial_port: None,
            _update_callback: None,
            _map_updated_event: Arc::new(Mutex::new(())),
            radar_unpack_fmt: "<HH".to_string() + &"HB".repeat(12) + "HH",
            thread_list: Vec::new(),
        }
    }

    fn start_serial_task(&mut self, com_port: &str, radar_type: &str) {
        if self.serial_running.load(Ordering::SeqCst) {
            self.serial_stop(false);
        }

        let baudrate = match radar_type {
            "LD08" => 115200,
            "LD06" => 230400,
            _ => panic!("Unknown radar type"),
        };

        let settings = SerialPortSettings {
            baud_rate: baudrate,
            timeout: Duration::from_millis(10),
            ..Default::default()
        };

        match serialport::open_with_settings(com_port, &settings) {
            Ok(port) => self.serial_port = Some(Box::new(port)),
            Err(e) => panic!("Failed to open port: {}", e),
        }

        self.serial_running.store(true, Ordering::SeqCst);

        let serial_running = self.serial_running.clone();
        let serial_port = self.serial_port.as_mut().unwrap();
        let package = Arc::new(Mutex::new(self.package.clone()));
        let base = Arc::new(Mutex::new(self.base.clone()));
        let map_updated_event = self._map_updated_event.clone();

        let handle = thread::spawn(move || {
            Self::read

_serial_task(
                serial_running,
                serial_port,
                package,
                base,
                map_updated_event,
            )
        });

        self.thread_list.push(handle);
        println!("[radar] serial task start");
    }

    fn serial_stop(&mut self, joined: bool) {
        self.serial_running.store(false, Ordering::SeqCst);
        if joined {
            if let Some(handle) = self.thread_list.pop() {
                handle.join().unwrap();
            }
        }
        if let Some(port) = &mut self.serial_port {
            port.close().unwrap();
        }
        println!("[RADAR] Stopped serial threads");
    }

    fn read_serial_task(
        serial_running: Arc<AtomicBool>,
        serial_port: &mut Box<dyn SerialPort>,
        package: Arc<Mutex<RadarPackage>>,
        base: Arc<Mutex<Map360>>,
        map_updated_event: Arc<Mutex<()>>,
    ) {
        let start_bit = [0x54, 0x2C];
        let package_length = 45;
        let mut read_buffer = vec![];
        let mut wait_buffer = vec![];

        while serial_running.load(Ordering::SeqCst) {
            if let Ok(bytes_available) = serial_port.bytes_to_read() {
                if bytes_available > 0 {
                    if read_buffer.is_empty() {
                        let mut byte = [0];
                        serial_port.read_exact(&mut byte).unwrap();
                        wait_buffer.push(byte[0]);

                        if wait_buffer.len() >= 2 {
                            if wait_buffer.ends_with(&start_bit) {
                                read_buffer.extend_from_slice(&start_bit);
                                wait_buffer.clear();
                            }
                        }
                    } else {
                        let mut data = vec![0; package_length];
                        serial_port.read_exact(&mut data).unwrap();
                        read_buffer.extend_from_slice(&data);

                        let mut pkg = package.lock().unwrap();
                        *pkg = RadarSerialUpdater::resolve_radar_data(&read_buffer, &pkg);
                        read_buffer.clear();

                        if pkg.recent_update_result {
                            let mut base_lock = base.lock().unwrap();
                            base_lock.update(&pkg);
                            map_updated_event.lock().unwrap();
                        } else {
                            println!("[RADAR] Map Update Error");
                        }
                    }
                }
            } else {
                thread::sleep(Duration::from_millis(1));
            }
        }
    }

    fn resolve_radar_data(data: &[u8], to_package: &RadarPackage) -> RadarPackage {
        if data.len() != 47 {
            println!("[RADAR] Invalid data length: {}", data.len());
            return RadarPackage::new(None);
        }

        if Self::calculate_crc8(&data[..46]) != data[46] {
            println!("[RADAR] Invalid CRC8");
            return RadarPackage::new(None);
        }

        if &data[..2] != &[0x54, 0x2C] {
            println!("[RADAR] Invalid header: {:?}", &data[..2]);
            return RadarPackage::new(None);
        }

        let datas: Vec<u32> = data[2..46]
            .chunks_exact(2)
            .map(|chunk| u32::from_le_bytes([chunk[0], chunk[1], 0, 0]))
            .collect();

        let mut new_package = to_package.clone();
        new_package.fill_data(&datas);
        new_package.recent_update_result = true;

        new_package
    }

    fn calculate_crc8(data: &[u8]) -> u8 {
        const CRC_TABLE: [u8; 256] = [
            0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
            0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
            0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
            0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
            0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
            0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
            0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
            0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
            0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
            0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
            0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
            0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
            0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
            0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
            0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 

0x78, 0xD6, 0x9B, 0x4C, 0x01,
            0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
        ];

        let mut crc = 0x00;
        for &byte in data {
            crc = CRC_TABLE[(crc ^ byte) as usize];
        }
        crc
    }
}

struct Radar {
    com_port: Option<String>,
    radar_type: Option<String>,
    updater: RadarSerialUpdater,
}

impl Radar {
    fn new() -> Self {
        Radar {
            com_port: None,
            radar_type: None,
            updater: RadarSerialUpdater::new(),
        }
    }

    fn start(&mut self, com_port: String, radar_type: String) {
        // 开始监听串口
        self.com_port = Some(com_port);
        self.radar_type = Some(radar_type);
        self.updater.start_serial_task(
            self.com_port.clone().unwrap(),
            self.radar_type.clone().unwrap(),
        );
    }
}

fn main() {
    let mut radar = Radar::new();
    radar.start("/dev/ttyUSB0".to_string(), "LD06".to_string());
}
