use std::f64;
use std::env;

fn rad_to_deg(radians: f64) -> f64 {
    180. / f64::consts::PI * radians
}

fn normalize_angle(angle: f64) -> f64 {
    let mut a: f64 = angle % f64::consts::TAU;
    if a > f64::consts::PI {
        a -= f64::consts::TAU;
    } else if a <= -f64::consts::PI {
        a += f64::consts::TAU;
    }
    a
}

fn inverse_kinematics(x: f64, y: f64, side: bool) -> (f64, f64) {
    let y: f64 = y.min(-0.1);
    let mut temp_0: f64 = (x * x + y * y).sqrt();
    temp_0 = temp_0.clamp(0.4, 1.9);
    let temp_1: f64 = ((2. - temp_0 * temp_0) / 2.).acos();
    let temp_2: f64 = normalize_angle(y.atan2(x) - (temp_0 / 2.).acos());
    let mut temp_3: f64 = normalize_angle(temp_2 - temp_1);
    let mut temp_4: f64 = normalize_angle(f64::consts::PI + temp_2);

    if !side {
        temp_3 = temp_3.clamp(f64::consts::FRAC_PI_2, f64::consts::PI);
        temp_4 = temp_4.clamp(0., f64::consts::FRAC_PI_2);

        (temp_3, temp_4)
    } else {
        temp_3 = temp_3.clamp(0., f64::consts::FRAC_PI_2);
        temp_4 = temp_4.clamp(f64::consts::FRAC_PI_2, f64::consts::PI);

        (temp_4, temp_3)
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    let x: f64 = args[1].parse::<f64>().unwrap();
    let y: f64 = args[2].parse::<f64>().unwrap();

    let angles: (f64, f64) = inverse_kinematics(x, y, false);
    println!("angle 1: {:.2}, angle 2: {:.2}", rad_to_deg(angles.0), rad_to_deg(angles.1));
}
