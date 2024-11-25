#![no_std]
#![no_main]
#![feature(str_from_raw_parts)]

use bouffalo_hal::{prelude::*, spi::Spi, uart::Config as UartConfig};
use bouffalo_rt::{entry, Clocks, Peripherals};
use core::ptr;
use embedded_cli::{cli::CliBuilder, Command};
use embedded_hal::{digital::OutputPin, spi::MODE_3};
use embedded_io::Write;
use embedded_sdmmc::{Mode, SdCard, VolumeManager};
use embedded_time::rate::*;
use panic_halt as _;

struct MyTimeSource {}

impl embedded_sdmmc::TimeSource for MyTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        // TODO
        embedded_sdmmc::Timestamp::from_calendar(2023, 1, 1, 0, 0, 0).unwrap()
    }
}

struct Device<
    'a,
    W: Write,
    R: Read,
    L: OutputPin,
    SPI: core::ops::Deref<Target = bouffalo_hal::spi::RegisterBlock>,
    PADS,
    const I: usize,
> {
    tx: &'a mut W,
    rx: &'a mut R,
    led: &'a mut L,
    volume_mgr: VolumeManager<SdCard<Spi<SPI, PADS, I>, riscv::delay::McycleDelay>, MyTimeSource>,
}

impl<
        'a,
        W: Write,
        R: Read,
        L: OutputPin,
        SPI: core::ops::Deref<Target = bouffalo_hal::spi::RegisterBlock>,
        PADS,
        const I: usize,
    > Device<'a, W, R, L, SPI, PADS, I>
{
    /// Create a new device.
    pub fn new(tx: &'a mut W, rx: &'a mut R, led: &'a mut L, spi: Spi<SPI, PADS, I>) -> Self {
        // Initialize sdcard.
        let delay = riscv::delay::McycleDelay::new(40_000_000);
        let sdcard = SdCard::new(spi, delay);
        while sdcard.get_card_type().is_none() {
            core::hint::spin_loop();
        }
        writeln!(
            tx,
            "Card size: {:.2}GB",
            sdcard.num_bytes().unwrap() as f32 / (1024.0 * 1024.0 * 1024.0)
        )
        .ok();
        let volume_mgr = VolumeManager::new(sdcard, MyTimeSource {});
        Self {
            tx,
            rx,
            led,
            volume_mgr,
        }
    }

    /// Load the SD card
    pub fn load_from_sdcard(&mut self, config: &mut Config) -> bool {
        let mut state = true;
        // Initialize filesystem.
        let volume0 = self
            .volume_mgr
            .open_raw_volume(embedded_sdmmc::VolumeIdx(0))
            .unwrap();
        let root_dir = self.volume_mgr.open_root_dir(volume0).unwrap();

        self.volume_mgr
            .iterate_dir(root_dir, |entry| {
                writeln!(self.tx, "Entry: {:?}", entry).ok();
            })
            .unwrap();

        writeln!(self.tx, "Filesystem initialized.").ok();

        // Read configuration from `config.toml`.
        let bl808_cfg = "CONFIG~1.TOM";
        if self
            .volume_mgr
            .find_directory_entry(root_dir, bl808_cfg)
            .is_err()
        {
            state = false;
            writeln!(self.tx, "warning: cannot find config file `/config.toml`, using default configuration from DTB file.").ok();
        }
        let config_file = self
            .volume_mgr
            .open_file_in_dir(root_dir, bl808_cfg, Mode::ReadOnly)
            .unwrap();
        // Read config from raw file.
        let buffer = &mut [0u8; 128];
        self.volume_mgr.read(config_file, buffer).ok();
        config.bootargs = *buffer;

        let bootargs = core::str::from_utf8(&config.bootargs).unwrap();

        writeln!(self.tx, "----raw-bootargs----").ok();
        for i in 0..config.bootargs.len() {
            if config.bootargs[i] != 0 {
                write!(self.tx, "{}", config.bootargs[i] as char).ok();
            }
        }
        writeln!(self.tx, "----coverted-bootargs----").ok();

        writeln!(self.tx, "Bootargs: {}", bootargs).ok();

        self.volume_mgr.close_file(config_file).ok();

        // Load `bl808.dtb` to memory.
        let bl808_dtb = "HWDTB~1.5M";
        let dtb_addr = 0x51ff_8000;
        if self
            .volume_mgr
            .find_directory_entry(root_dir, bl808_dtb)
            .is_err()
        {
            state = false;
            writeln!(
                self.tx,
                "error: cannot find device tree blob file `bl808.dtb`, aborting bootload process.
"
            )
            .ok();
        }
        let dtb_file = self
            .volume_mgr
            .open_file_in_dir(root_dir, bl808_dtb, Mode::ReadOnly)
            .unwrap();
        let dtb_file_size = self.volume_mgr.file_length(dtb_file).unwrap() as f32 / 1024.0;
        if dtb_file_size > 64.0 {
            state = false;
            writeln!(self.tx, "error: /bl808.dtb: file size is {:.2} KB, but maximum supported device tree blob size is 64KB.", dtb_file_size).ok();
        } else {
            let mut buffer = &mut [0u8; 64 * 1024];
            self.volume_mgr.read(dtb_file, buffer);
        }
        // core::mem::replace(dest, src);

        self.volume_mgr.close_file(dtb_file).ok();
        // Load `zImage` to memory.
        let bl808_zImg = "HWDTB~1.5M";
        let z_image_addr = 0x5000_0000;
        if self
            .volume_mgr
            .find_directory_entry(root_dir, bl808_zImg)
            .is_err()
        {
            state = false;
            writeln!(
                self.tx,
                "error: cannot find device tree blob file `bl808_zImg`, aborting bootload process.
"
            )
            .ok();
        }
        let z_image_file = self
            .volume_mgr
            .open_file_in_dir(root_dir, bl808_zImg, Mode::ReadOnly)
            .unwrap();

        let z_image_file_size =
            self.volume_mgr.file_length(z_image_file).unwrap() as f32 / (1024.0 * 1024.0);
        if z_image_file_size > 31.96875 {
            state = false;
            writeln!(
                self.tx,
                "error: /bl808.zImg: file size is {:.5} MB, but maximum supported zImage size is 31.96875MB.", z_image_file_size).ok();
        } else {
            let buffer = &mut [0u8; 64 * 1024];
            self.volume_mgr.read(dtb_file, buffer);
        }

        self.volume_mgr.close_file(z_image_file).ok();

        self.volume_mgr.close_dir(root_dir).unwrap();

        writeln!(self.tx, "load_from_sdcard success").ok();
        state
    }

    pub fn run_cli(&mut self, config: &mut Config) -> ! {
        #[derive(Command)]
        enum Base {
            /// Print out 'Hello world!'.
            Hello,
            /// LED control command.
            Led {
                #[command(subcommand)]
                command: Option<LedCommand>,
            },
        }

        #[derive(Command)]
        enum LedCommand {
            /// Turn on LED.
            On,
            /// Turn off LED.
            Off,
            /// Switch LED state.
            Switch,
        }

        // TODO: more commands.

        writeln!(self.tx, "Welcome to bouffaloader-cli!").ok();
        writeln!(self.tx, "For command helps, type 'help'.").ok();

        let (command_buffer, history_buffer) = ([0; 128], [0; 128]);
        let writer = unsafe { ptr::read(&self.tx as *const _) };
        let mut cli = CliBuilder::default()
            .writer(writer)
            .command_buffer(command_buffer)
            .history_buffer(history_buffer)
            .prompt("> ")
            .build()
            .unwrap();

        let mut led_state = PinState::Low;
        loop {
            self.led.set_state(led_state).ok();
            let mut slice = [0];
            self.rx.read_exact(&mut slice).ok();
            let _ = cli.process_byte::<Base, _>(
                slice[0],
                &mut Base::processor(|cli, command| {
                    match command {
                        Base::Hello => {
                            writeln!(self.tx, "Hello world!").ok();
                        }
                        Base::Led { command } => match command {
                            Some(LedCommand::On) => led_state = PinState::Low,
                            Some(LedCommand::Off) => led_state = PinState::High,
                            Some(LedCommand::Switch) => led_state = !led_state,
                            None => match led_state {
                                PinState::High => cli.writer().write_str("LED state: Off").unwrap(),
                                PinState::Low => cli.writer().write_str("LED state: On").unwrap(),
                            },
                        },
                    }
                    Ok(())
                }),
            );
        }
    }
}

struct Config {
    bootargs: [u8; 128],
}

#[entry]
fn main(p: Peripherals, c: Clocks) -> ! {
    // Initialize devices.
    let (tx, rx) = {
        let tx = p.gpio.io14.into_uart();
        let rx = p.gpio.io15.into_uart();
        let sig2 = p.uart_muxes.sig2.into_transmit::<0>();
        let sig3 = p.uart_muxes.sig3.into_receive::<0>();

        let config = UartConfig::default().set_baudrate(2000000.Bd());
        let serial = p.uart0.freerun(config, ((tx, sig2), (rx, sig3)), &c);

        &mut serial.split()
    };
    let led = &mut p.gpio.io8.into_floating_output();
    let spi = {
        let spi_clk = p.gpio.io3.into_spi::<1>();
        let spi_mosi = p.gpio.io1.into_spi::<1>();
        let spi_miso = p.gpio.io2.into_spi::<1>();
        let spi_cs = p.gpio.io0.into_spi::<1>();
        Spi::new(
            p.spi1,
            (spi_clk, spi_mosi, spi_miso, spi_cs),
            MODE_3,
            &p.glb,
        )
    };

    // Display bouffaloader banner.
    // TODO
    writeln!(tx, "Welcome to bouffaloader🦀!").ok();

    // Initialize PSRAM.
    // TODO

    // Initialize sdcard and load files.
    let mut d = Device::new(tx, rx, led, spi);
    let mut config = Config { bootargs: [0; 128] };
    if !d.load_from_sdcard(&mut config) {
        d.run_cli(&mut config);
    }

    // Skip run_payload if both buttons are pressed.
    let mut button_1 = p.gpio.io22.into_pull_up_input();
    let mut button_2 = p.gpio.io23.into_pull_up_input();
    let button_1_pressed = button_1.is_low().unwrap();
    let button_2_pressed = button_2.is_low().unwrap();
    if button_1_pressed && button_2_pressed {
        d.run_cli(&mut config);
    };

    // Run payload.
    run_payload();
}

fn run_payload() -> ! {
    const ZIMAGE_ADDRESS: usize = 0x5000_0000; // Load address of Linux zImage
    const DTB_ADDRESS: usize = 0x51FF_8000; // Address of the device tree blob
    const HART_ID: usize = 0; // Hartid of the current core

    type KernelEntry = unsafe extern "C" fn(hart_id: usize, dtb_addr: usize);

    let kernel_entry: KernelEntry = unsafe { core::mem::transmute(ZIMAGE_ADDRESS) };
    unsafe {
        kernel_entry(HART_ID, DTB_ADDRESS);
    }

    loop {}
}
