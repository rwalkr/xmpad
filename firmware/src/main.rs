#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::{PIO0, USB, I2C0};
use embassy_rp::{i2c, pio, usb};
use embedded_hal::i2c::I2c;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{block_for, Duration, Instant, Ticker, Timer};
use embassy_usb::class::hid::{
    self, HidReader, HidReaderWriter, HidWriter, ReportId, RequestHandler, State,
};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};
use packed_struct::prelude::*;
use static_cell::StaticCell;
use usbd_human_interface_device::device::mouse::{WHEEL_MOUSE_REPORT_DESCRIPTOR, WheelMouseReport};
use usbd_human_interface_device::{
    device::keyboard::{BootKeyboardReport, BOOT_KEYBOARD_REPORT_DESCRIPTOR},
    page::Keyboard,
};
use heapless::Vec;

use {defmt_rtt as _, panic_probe as _};

type KbdHidReaderWriter<'a> = HidReaderWriter<'a, usb::Driver<'a, USB>, 1, 25>;
type KbdHidReader<'a> = HidReader<'a, usb::Driver<'a, USB>, 1>;
type KbdHidWriter<'a> = HidWriter<'a, usb::Driver<'a, USB>, 25>;
type MouseHidWriter<'a> = HidWriter<'a, usb::Driver<'a, USB>, 5>;

#[derive(PartialEq)]
enum KbdEvent {
    Press,
    Release
}

type KbdEventQueue = Channel<ThreadModeRawMutex, KbdEvent, 4>;
type KbdEventQueueSender<'a> = Sender<'a, ThreadModeRawMutex, KbdEvent, 4>;
type KbdEventQueueReceiver<'a> = Receiver<'a, ThreadModeRawMutex, KbdEvent, 4>;
type KbdReportQueue = Channel<ThreadModeRawMutex, Keycodes, 4>;
type KbdReportQueueSender<'a> = Sender<'a, ThreadModeRawMutex, Keycodes, 4>;
type KbdReportQueueReceiver<'a> = Receiver<'a, ThreadModeRawMutex, Keycodes, 4>;
type MouseReportQueue = Channel<ThreadModeRawMutex, WheelMouseReport, 4>;
type MouseReportQueueSender<'a> = Sender<'a, ThreadModeRawMutex, WheelMouseReport, 4>;
type MouseReportQueueReceiver<'a> = Receiver<'a, ThreadModeRawMutex, WheelMouseReport, 4>;

mod modifiers {
    pub const SHIFT: u32 = 1;
    pub const CTRL: u32 = 2;
    pub const ALT: u32 = 4;
    pub const GUI: u32 = 8;
}
type Modifiers = u32;

enum KeyAction {
    Simple{ key: Keyboard, m: Modifiers },
    StickyModifiers{ key: Keyboard, m: Modifiers, pre_delay: u64, release_delay: u64 }
}

const KEY_COUNT: usize = 6;
const KEY_ACTIONS: [KeyAction; KEY_COUNT] = [
    KeyAction::Simple { key: Keyboard::F12, m: modifiers::CTRL | modifiers::ALT },
    KeyAction::StickyModifiers { key: Keyboard::Tab, m:  modifiers::ALT, pre_delay: 0, release_delay: 500 },
    KeyAction::StickyModifiers { key: Keyboard::P, m: modifiers::GUI, pre_delay: 50, release_delay: 1000 },
    KeyAction::Simple { key: Keyboard::ReturnEnter, m: 0 },
    KeyAction::Simple { key: Keyboard::F10, m: modifiers::CTRL | modifiers::ALT },
    KeyAction::Simple { key: Keyboard::F11, m: modifiers::CTRL | modifiers::ALT },
];

const MAX_KEYS: usize = KEY_COUNT + 4;
type Keycodes = Vec<Keyboard, MAX_KEYS>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let usb_driver = usb::Driver::new(p.USB, Irqs);

    let key_pins: [Input; KEY_COUNT] = [
        Input::new(p.PIN_2, Pull::Up),
        Input::new(p.PIN_3, Pull::Up),
        Input::new(p.PIN_4, Pull::Up),
        Input::new(p.PIN_5, Pull::Up),
        Input::new(p.PIN_6, Pull::Up),
        Input::new(p.PIN_7, Pull::Up),
    ];

    // Configure I²C interface
    let sda_pin = p.PIN_12;
    let scl_pin = p.PIN_13;
    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = 400_000u32;
    let i2c = i2c::I2c::new_blocking(p.I2C0, scl_pin, sda_pin, i2c_cfg);

    // Create embassy-usb Config
    // Test PID from https://pid.codes/1209/
    const VID: u16 = 0x1209;
    const PID: u16 = 0x0002;
    let mut config = Config::new(VID, PID);
    config.manufacturer = Some("rwalkr");
    config.product = Some("xmpad");
    config.serial_number = Some(env!("CARGO_PKG_VERSION"));
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 256]);
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let msos_descriptor = MSOS_DESCRIPTOR.init([0; 256]);
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; 64]);

    static KBD_HANDLER: StaticCell<KeyboardHandler> = StaticCell::new();
    let kbd_handler = KBD_HANDLER.init(KeyboardHandler::new());

    static DEVICE_HANDLER: StaticCell<USBDeviceHandler> = StaticCell::new();
    let device_handler = DEVICE_HANDLER.init(USBDeviceHandler::new());

    static KBD_HID_STATE: StaticCell<State> = StaticCell::new();
    let kbd_hid_state = KBD_HID_STATE.init(State::new());

    static MOUSE_HID_STATE: StaticCell<State> = StaticCell::new();
    let mouse_hid_state = MOUSE_HID_STATE.init(State::new());

    let mut builder = Builder::new(
        usb_driver,
        config,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
    );
    builder.handler(device_handler);

    // Create classes on the builder.
    let kbd_hid_config = hid::Config {
        report_descriptor: BOOT_KEYBOARD_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let kbd_hid = KbdHidReaderWriter::new(&mut builder, kbd_hid_state, kbd_hid_config);
    let (kbd_hid_reader, kbd_hid_writer) = kbd_hid.split();

    let mouse_hid_config = hid::Config {
        report_descriptor: WHEEL_MOUSE_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let mouse_hid = MouseHidWriter::new(&mut builder, mouse_hid_state, mouse_hid_config);

    static KBD_EVENT_QUEUE: StaticCell<Vec<KbdEventQueue, KEY_COUNT>> = StaticCell::new();
    let kbd_event_queue = KBD_EVENT_QUEUE.init((0..KEY_COUNT).map(|_| Channel::new()).collect());
    let kbd_event_senders: Vec<KbdEventQueueSender, KEY_COUNT> = kbd_event_queue.iter().map(|q| q.sender()).collect();
    let kbd_event_receivers: Vec<KbdEventQueueReceiver, KEY_COUNT> = kbd_event_queue.iter().map(|q| q.receiver()).collect();

    static KBD_REPORT_QUEUE: StaticCell<KbdReportQueue> = StaticCell::new();
    let kbd_report_queue = KBD_REPORT_QUEUE.init(Channel::new());
    let kbd_report_sender = kbd_report_queue.sender();
    let kbd_report_receiver = kbd_report_queue.receiver();

    static MOUSE_REPORT_QUEUE: StaticCell<MouseReportQueue> = StaticCell::new();
    let mouse_report_queue = MOUSE_REPORT_QUEUE.init(Channel::new());
    let mouse_report_sender = mouse_report_queue.sender();
    let mouse_report_receiver = mouse_report_queue.receiver();

    spawner.must_spawn(kbd_scan(key_pins, kbd_event_senders));
    for i in 0..KEY_COUNT {
        spawner.must_spawn(kbd_key_handler(&KEY_ACTIONS[i], kbd_event_receivers[i], kbd_report_sender));
    }
    spawner.must_spawn(kbd_hid_write(kbd_report_receiver, kbd_hid_writer));
    spawner.must_spawn(kbd_hid_read(kbd_hid_reader, kbd_handler));

    spawner.must_spawn(scroll_wheel(i2c, mouse_report_sender));
    spawner.must_spawn(mouse_hid_write(mouse_report_receiver, mouse_hid));

    let mut usb = builder.build();
    usb.run().await;
}

struct KeyboardHandler {}

impl RequestHandler for KeyboardHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

impl KeyboardHandler {
    fn new() -> KeyboardHandler {
        KeyboardHandler {}
    }
}

#[embassy_executor::task]
async fn kbd_scan(
    key_pins: [Input<'static>; KEY_COUNT],
    kbd_event_queues: Vec<KbdEventQueueSender<'static>, KEY_COUNT>
) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    const DEBOUNCE_TICKS: usize = 5;

    let mut pin_states = [false; KEY_COUNT];
    let mut pin_state_counts = [0; KEY_COUNT];
    let mut key_states = [false; KEY_COUNT];
    loop {
        // read pins and update states
        for p in 0..KEY_COUNT {
            let cur_state = key_pins[p].is_low();
            if pin_states[p] != cur_state {
                // pin has changed since last reading - reset counter
                pin_states[p] = cur_state;
                pin_state_counts[p] = 1;
            } else {
                // not changed
                pin_state_counts[p] += 1;
                if pin_state_counts[p] == DEBOUNCE_TICKS {
                    // steady state - update key state
                    info!("kbd_scan: Detected pin {}: {}", p, pin_states[p]);
                    key_states[p] = pin_states[p];
                    let ev = if pin_states[p] {
                        KbdEvent::Press
                    } else  {
                        KbdEvent::Release
                    };
                    kbd_event_queues[p].send(ev).await;
                }
            }
        }

        ticker.next().await;
    }
}

async fn wait_for_press(kbd_event_queue: &KbdEventQueueReceiver<'static>) {
    loop {
        let ev = kbd_event_queue.receive().await;
        if ev == KbdEvent::Press {
            break
        }
    }
}

async fn wait_for_release(kbd_event_queue: &KbdEventQueueReceiver<'static>) {
    loop {
        let ev = kbd_event_queue.receive().await;
        if ev == KbdEvent::Release {
            break
        }
    }
}

fn apply_modifiers(keycodes: &mut Keycodes, m: &Modifiers) {
    if m & modifiers::SHIFT != 0 {
        keycodes.push(Keyboard::LeftShift).unwrap()
    }
    if m & modifiers::CTRL != 0 {
        keycodes.push(Keyboard::LeftControl).unwrap()
    }
    if m & modifiers::ALT != 0 {
        keycodes.push(Keyboard::LeftAlt).unwrap()
    }
    if m & modifiers::GUI != 0 {
        keycodes.push(Keyboard::LeftGUI).unwrap()
    }
}

#[embassy_executor::task(pool_size = KEY_COUNT)]
async fn kbd_key_handler(
    kbd_action: &'static KeyAction,
    kbd_event_queue: KbdEventQueueReceiver<'static>,
    kbd_report_queue: KbdReportQueueSender<'static>,
) {
    match kbd_action {
        KeyAction::Simple{ key, m } => simple_key_handler(key, m, kbd_event_queue, kbd_report_queue).await,
        KeyAction::StickyModifiers { key, m, pre_delay, release_delay } => sticky_mod_key_handler(key, m, pre_delay, release_delay, kbd_event_queue, kbd_report_queue).await,
    }
}

async fn simple_key_handler(
    key: &Keyboard,
    m: &Modifiers,
    kbd_event_queue: KbdEventQueueReceiver<'static>,
    kbd_report_queue: KbdReportQueueSender<'static>,
) {
    loop {
        info!("kbd_state: Waiting for press");
        wait_for_press(&kbd_event_queue).await;

        // send key and modifiers
        let mut keycodes = Keycodes::new();
        keycodes.push(*key).unwrap_or_default();
        apply_modifiers(&mut keycodes, m);
        kbd_report_queue.send(keycodes).await;

        info!("kbd_state: Waiting for release");
        wait_for_release(&kbd_event_queue).await;

        // send empty key set
        kbd_report_queue.send(Keycodes::new()).await;
    }
}

async fn sticky_mod_key_handler(
    key: &Keyboard,
    m: &Modifiers,
    pre_delay: &u64,
    release_delay: &u64,
    kbd_event_queue: KbdEventQueueReceiver<'static>,
    kbd_report_queue: KbdReportQueueSender<'static>,
) {
    let mut mods_held_until = Instant::MAX;
    loop {
        info!("kbd_state: Waiting for press");
        loop {
            let timeout = Timer::at(mods_held_until);
            let ev = select(kbd_event_queue.receive(), timeout).await;
            match ev {
                Either::First(KbdEvent::Press) => {
                    break
                }
                Either::First(KbdEvent::Release) => {
                    // spurious release - keep waiting
                }
                Either::Second(_) => {
                    // deadline elapsed:
                    // send empty keycodes to release mods
                    kbd_report_queue.send(Keycodes::new()).await;
                    mods_held_until = Instant::MAX;
                    // continue waiting
                }
            }
        }

        let mut keycodes = Keycodes::new();
        apply_modifiers(&mut keycodes, m);

        // maybe send modifiers early
        if keycodes.len() > 0 && *pre_delay > 0 {
            kbd_report_queue.send(keycodes.clone()).await;
            block_for(Duration::from_millis(*pre_delay));
        }

        // send the key
        keycodes.push(*key).unwrap_or_default();
        kbd_report_queue.send(keycodes.clone()).await;

        info!("kbd_state: Waiting for release");
        wait_for_release(&kbd_event_queue).await;

        keycodes = Keycodes::new();
        if *release_delay > 0 {
            apply_modifiers(&mut keycodes, m);
            mods_held_until = Instant::now() + Duration::from_millis(*release_delay);
        } else {
            mods_held_until = Instant::MAX;
        }
        kbd_report_queue.send(keycodes.clone()).await;
    }
}

#[embassy_executor::task]
async fn kbd_hid_write(
    kbd_report_queue: KbdReportQueueReceiver<'static>,
    mut kbd_writer: KbdHidWriter<'static>,
) {
    loop {
        info!("hid_writer: Waiting for event");
        let keycodes = kbd_report_queue.receive().await;
        let report = BootKeyboardReport::new(keycodes);

        info!("hid_writer: Sending report");
        match kbd_writer.write(&report.pack().unwrap()).await {
            Ok(()) => {}
            Err(e) => warn!("Failed to send report: {:?}", e),
        };
    }
}

#[embassy_executor::task]
async fn kbd_hid_read(
    kbd_hid_reader: KbdHidReader<'static>,
    kbd_handler: &'static mut KeyboardHandler,
) {
    kbd_hid_reader.run(false, kbd_handler).await;
}

#[derive(Clone, Debug, Default)]
struct MouseReport(WheelMouseReport);

impl MouseReport {
    pub fn wheel(mut self, v: i8) -> Self {
        self.0.vertical_wheel = v;
        self
    }
}

impl From<MouseReport> for WheelMouseReport {
    fn from(r: MouseReport) -> Self {
        r.0
    }
}

fn read_reg_u8(i2c: &mut i2c::I2c<'static, I2C0, i2c::Blocking>, addr: u8, reg_num: u8) -> Result<u8, i2c::Error> {
    let mut rd_buf = [0xA5u8; 1];
    i2c.write_read(addr, &reg_num.to_be_bytes(), &mut rd_buf)?;
    let r = rd_buf[0];
    Ok(r)
}

fn read_reg_pair(i2c: &mut i2c::I2c<'static, I2C0, i2c::Blocking>, addr: u8, reg_nums: (u8, u8)) -> Result<(u8, u8), i2c::Error> {
    let r0 = read_reg_u8(i2c, addr, reg_nums.0)?;
    let r1 = read_reg_u8(i2c, addr, reg_nums.1)?;
    Ok((r0, r1))
}

fn read_scroll_pos(i2c: &mut i2c::I2c<'static, I2C0, i2c::Blocking>) -> Result<u16, i2c::Error> {
    const I2C_ADDR: u8 = 0x06;
    let (hi, lo) = read_reg_pair(i2c, I2C_ADDR, (0x03, 0x04))?;
    Ok(((hi as u16) << 6) | ((lo as u16) >> 2))
}

#[embassy_executor::task]
async fn scroll_wheel(
    mut i2c: i2c::I2c<'static, I2C0, i2c::Blocking>,
    mouse_report_sender: MouseReportQueueSender<'static>
) {
    info!("Starting scroll scan");
    let mut ticker = Ticker::every(Duration::from_millis(100));
    const STEPS_PER_REV: i16 = 16384;
    const STEPS_PER_SCROLL: i16 = 128;

    ticker.next().await;

    let mut last_pos: u16 = read_scroll_pos(&mut i2c).unwrap_or(0);
    let mut scroll_credit: i16 = 0;
    loop {
        match read_scroll_pos(&mut i2c) {
            Ok(pos) => {
                let mut delta = (pos as i16).wrapping_sub_unsigned(last_pos);
                if delta > STEPS_PER_REV/2 { 
                    delta -= STEPS_PER_REV;
                }
                if delta < -STEPS_PER_REV/2 {
                    delta += STEPS_PER_REV;
                }

                last_pos = pos;

                scroll_credit += delta as i16;
                let scroll = scroll_credit / STEPS_PER_SCROLL;
                scroll_credit -= scroll * STEPS_PER_SCROLL;
                if scroll != 0 {
                    mouse_report_sender.send(MouseReport::default().wheel(scroll as i8).into()).await;
                }

                info!("V = {} :: {} => {}", pos, delta, scroll);
            }
            Err(e) => {
                warn!("I2C read failed: {:?}", e);
            }
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn mouse_hid_write(
    mouse_report_queue: MouseReportQueueReceiver<'static>,
    mut mouse_writer: MouseHidWriter<'static>,
) {
    loop {
        info!("mouse hid_writer: Waiting for event");
        let report = mouse_report_queue.receive().await;

        info!("hid_writer: Sending report");
        match mouse_writer.write(&report.pack().unwrap()).await {
            Ok(()) => {}
            Err(e) => warn!("Failed to send report: {:?}", e),
        };
    }
}


struct USBDeviceHandler {
    configured: AtomicBool,
}

impl USBDeviceHandler {
    fn new() -> Self {
        USBDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for USBDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
