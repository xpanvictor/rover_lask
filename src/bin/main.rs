#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::ops::Add;
use core::pin::Pin;
use core::sync::atomic::AtomicU32;
use core::task::Poll;

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::rng::{Rng, Trng, TrngSource};
use esp_hal::system::{CpuControl, Stack};
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    error!("{}", panic_info);
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// core2
static CORE1_STACK: StaticCell<Stack<65536>> = StaticCell::new();
static EXECUTOR_C1: StaticCell<Executor> = StaticCell::new();

static WHEEL_SPEED: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
struct VehicleState {
    speed_mm_s: u32,
    steer: i16,
    motors_on: bool,
}

const VS_QUEUE_SIZE: usize = 5;
static VEHICLE_QUEUE: Channel<CriticalSectionRawMutex, VehicleState, VS_QUEUE_SIZE> =
    Channel::new();

#[derive(Clone, Copy, defmt::Format)]
struct Command {
    steer: i16,
    throttle: i16,
}

// command signal
static CMD_CORE: Signal<CriticalSectionRawMutex, Command> = Signal::new();

struct DelayMS {
    ms: u64,
    inner: Option<Timer>,
    deadline: Option<Instant>,
}

impl DelayMS {
    pub fn new(ms: u64) -> Self {
        Self {
            ms,
            inner: None,
            deadline: None,
        }
    }
}

impl Future for DelayMS {
    type Output = ();
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        if self.deadline.is_none() {
            let deadline = Instant::now() + Duration::from_millis(self.ms);
            self.deadline = Some(deadline);
            self.inner = Some(Timer::at(deadline));
        }

        if self.deadline.is_some_and(|d| d <= Instant::now()) {
            return Poll::Ready(());
        }

        // poll cx
        let inner = self.inner.as_mut().unwrap();
        let inner = unsafe { Pin::new_unchecked(inner) };
        match inner.poll(cx) {
            Poll::Pending => Poll::Pending,
            Poll::Ready(()) => Poll::Ready(()),
        }
    }
}

// macro_rules! mk_static {
//     ($t:ty, $val:expr) => {{
//         static CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
//         CELL.uninit().write($val)
//     }};
// }

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32 -o unstable-hal -o embassy -o defmt -o neovim -o vscode

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // Access core1
    let stack = CORE1_STACK.init(Stack::new());

    let core1_sw_intr = sw_interrupt.software_interrupt1;
    esp_rtos::start_second_core(peripherals.CPU_CTRL, core1_sw_intr, stack, core1_main);

    spawner.spawn(cmd_generator().unwrap());

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}

fn core1_main() {
    let executor_1 = EXECUTOR_C1.init(Executor::new());
    executor_1.run(|spawner| {
        spawner.spawn(control_loop().unwrap());
        spawner.spawn(speed_sensor().unwrap());
    })
}

#[embassy_executor::task]
async fn control_loop() {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        let cmd = CMD_CORE.wait().await;
        let _ = apply_speed(cmd.throttle);
        let speed = WHEEL_SPEED.load(core::sync::atomic::Ordering::Relaxed);
        let vs = VehicleState {
            steer: 0,
            speed_mm_s: speed,
            motors_on: true,
        };
        VEHICLE_QUEUE.send(vs).await;
        ticker.next().await
    }
}

fn apply_speed(speed: i16) -> u32 {
    WHEEL_SPEED.update(
        core::sync::atomic::Ordering::SeqCst,
        core::sync::atomic::Ordering::SeqCst,
        |sp| 0.max(speed.add(sp as i16) as u32).min(100),
    );
    WHEEL_SPEED.load(core::sync::atomic::Ordering::Relaxed)
}

#[embassy_executor::task]
async fn speed_sensor() {
    loop {
        DelayMS::new(20).await;
        let speed = WHEEL_SPEED.load(core::sync::atomic::Ordering::Relaxed);
        defmt::info!("speed: {}", speed);
        embassy_futures::yield_now().await;
    }
}

#[embassy_executor::task]
async fn cmd_generator() {
    let mut ticker = Ticker::every(Duration::from_millis(200));
    let rng = Rng::new();
    loop {
        let t_rand = rng.random();
        let (t_min, t_max) = (-20, 20); // inc or dec max 20
        let (s_min, s_max) = (-15, 15);
        let cmd = Command {
            throttle: random_calc(t_rand, t_min, t_max),
            steer: random_calc(t_rand, s_min, s_max),
        };
        defmt::info!("cmd: {:#?}", cmd);
        CMD_CORE.signal(cmd);
        ticker.next().await
    }
}

fn random_calc(r: u32, min: i16, max: i16) -> i16 {
    let delta = max - min;
    min + (r % delta as u32) as i16
}
