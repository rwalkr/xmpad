#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{AnyPin, Input, Pull};
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::{pio, usb};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Ticker};
use embassy_usb::class::hid::{
    self, HidReader, HidReaderWriter, HidWriter, ReportId, RequestHandler, State,
};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};
use packed_struct::prelude::*;
use static_cell::StaticCell;
use usbd_human_interface_device::{
    device::keyboard::{BootKeyboardReport, BOOT_KEYBOARD_REPORT_DESCRIPTOR},
    page::Keyboard,
};

use {defmt_rtt as _, panic_probe as _};

type KbdHidReaderWriter<'a> = HidReaderWriter<'a, usb::Driver<'a, USB>, 1, 25>;
type KbdHidReader<'a> = HidReader<'a, usb::Driver<'a, USB>, 1>;
type KbdHidWriter<'a> = HidWriter<'a, usb::Driver<'a, USB>, 25>;

type KbdReportQueue = Channel<ThreadModeRawMutex, Keycodes, 4>;

type KeyEvents = [Keyboard; 3];
const NK: Keyboard = Keyboard::NoEventIndicated;

const KEY_COUNT: usize = 6;
const KEY_CODES: [KeyEvents; KEY_COUNT] = [
    [Keyboard::LeftAlt, Keyboard::LeftControl, Keyboard::F12], // Stop
    [Keyboard::LeftAlt, Keyboard::Tab, NK], // 1L
    [Keyboard::LeftAlt, Keyboard::F4, NK], // 1M
    [Keyboard::ReturnEnter, NK, NK],            // 1R,
    [Keyboard::LeftAlt, Keyboard::LeftControl, Keyboard::F10], // 2L
    [Keyboard::LeftAlt, Keyboard::LeftControl, Keyboard::F11], // 2R
];
type Keycodes = heapless::Vec<Keyboard, KEY_COUNT>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let usb_driver = usb::Driver::new(p.USB, Irqs);

    let key_pins: [Input<AnyPin>; KEY_COUNT] = [
        Input::new(AnyPin::from(p.PIN_2), Pull::Up),
        Input::new(AnyPin::from(p.PIN_3), Pull::Up),
        Input::new(AnyPin::from(p.PIN_4), Pull::Up),
        Input::new(AnyPin::from(p.PIN_5), Pull::Up),
        Input::new(AnyPin::from(p.PIN_6), Pull::Up),
        Input::new(AnyPin::from(p.PIN_7), Pull::Up),
    ];

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
    static DEVICE_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let device_descriptor = DEVICE_DESCRIPTOR.init([0; 256]);
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

    let mut builder = Builder::new(
        usb_driver,
        config,
        device_descriptor,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
    );
    builder.handler(device_handler);

    // Create classes on the builder.
    let kbd_hid_config = hid::Config {
        report_descriptor: BOOT_KEYBOARD_REPORT_DESCRIPTOR,
        request_handler: Some(kbd_handler),
        poll_ms: 60,
        max_packet_size: 64,
    };
    let kbd_hid = KbdHidReaderWriter::new(&mut builder, kbd_hid_state, kbd_hid_config);

    static KBD_REPORT_QUEUE: StaticCell<KbdReportQueue> = StaticCell::new();
    let kbd_report_queue = KBD_REPORT_QUEUE.init(Channel::new());

    let (kbd_hid_reader, kbd_hid_writer) = kbd_hid.split();
    spawner.must_spawn(kbd_scan(key_pins, kbd_report_queue.sender()));
    spawner.must_spawn(kbd_hid_write(kbd_report_queue.receiver(), kbd_hid_writer));
    spawner.must_spawn(kbd_hid_read(kbd_hid_reader, kbd_handler));

    let mut usb = builder.build();
    usb.run().await;
}

struct KeyboardHandler {}

impl RequestHandler for KeyboardHandler {
    fn get_report(&self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&self, id: Option<ReportId>) -> Option<u32> {
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
    key_pins: [Input<'static, AnyPin>; KEY_COUNT],
    kbd_report_queue: Sender<'static, ThreadModeRawMutex, Keycodes, 4>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    const DEBOUNCE_TICKS: usize = 5;

    let mut pin_states = [false; KEY_COUNT];
    let mut pin_state_counts = [0; KEY_COUNT];
    let mut key_states = [false; KEY_COUNT];
    loop {
        // read pins and update states
        let mut key_changed = false;
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
                    key_changed = true;
                }
            }
        }

        // send report
        if key_changed {
            let mut keycodes = Keycodes::new();
            for (i, p) in key_states.iter().enumerate() {
                if *p {
                    for k in KEY_CODES[i] {
                        keycodes.push(k).unwrap_or_default();
                    }
                }
            }

            info!("kbd_scan: report {}", keycodes.len());
            kbd_report_queue.send(keycodes.clone()).await;
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn kbd_hid_write(
    kbd_report_queue: Receiver<'static, ThreadModeRawMutex, Keycodes, 4>,
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
    kbd_handler: &'static KeyboardHandler,
) {
    kbd_hid_reader.run(false, kbd_handler).await;
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
