static mut LAUNCH_COUNT: u32 = 0;
static mut ACCURATE_COUNT: u32 = 0;

pub fn increase_launch() {
    unsafe {
        LAUNCH_COUNT += 1;
    }
}

pub fn launch_count() -> u32 {
    unsafe { LAUNCH_COUNT }
}

pub fn increase_accurate() {
    unsafe {
        ACCURATE_COUNT += 1;
    }
}

pub fn accurate_count() -> u32 {
    unsafe { ACCURATE_COUNT }
}

pub fn accurate_pct() -> f32 {
    let l = launch_count();
    if l == 0 {
        return 0.0;
    }
    (accurate_count() as f32) / (launch_count() as f32)
}
