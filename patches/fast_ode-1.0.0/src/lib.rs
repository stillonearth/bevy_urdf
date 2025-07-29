#![doc = include_str!("../README.md")]
#![cfg_attr(test, feature(test))]
#![allow(clippy::many_single_char_names)] // That's ok, because we are doing numerics here

#[cfg(test)]
extern crate assert_approx_eq;
extern crate float_next_after;
#[cfg(test)]
extern crate mathru;
#[cfg(test)]
extern crate test;

use float_next_after::NextAfter;

#[cfg(test)]
mod optimizer_tests;
#[cfg(test)]
mod unit_tests;

pub trait DifferentialEquation<const N: usize> {
    /// The differential equation is
    /// dy/dt = ode_dot_y(t, y).0    ∀ t ∈ ℝ.
    ///
    /// If you do not like your argument, you have two choices:
    ///
    /// If you return `(Coord::<N>([NAN; N]), b)` you get a smaller step,
    /// independent of `b`.
    ///
    /// If you return `(c, false)`, the solver will terminate if (t, y) is part
    /// of the solution curve. In this case, [IvpResult::OdeRequestedExit] is
    /// returned.
    fn ode_dot_y(&self, t: f64, y: &Coord<{ N }>) -> (Coord<{ N }>, bool);
}

#[derive(Clone, Copy, Debug)]
pub struct Coord<const N: usize>(pub [f64; N]);

impl<const N: usize> core::ops::Mul<f64> for Coord<{ N }> {
    type Output = Coord<{ N }>;
    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        let mut z = [0.; N];
        for (zref, aval) in z.iter_mut().zip(&self.0) {
            *zref = aval * rhs;
        }
        Coord::<{ N }>(z)
    }
}

impl<const N: usize> core::ops::Mul<f64> for &Coord<{ N }> {
    type Output = Coord<{ N }>;
    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        let mut z = [0.; N];
        for (zref, aval) in z.iter_mut().zip(&self.0) {
            *zref = aval * rhs;
        }
        Coord::<{ N }>(z)
    }
}

impl<const N: usize> core::ops::Add<Coord<{ N }>> for &Coord<{ N }> {
    type Output = Coord<{ N }>;
    #[inline]
    fn add(self, rhs: Coord<{ N }>) -> Self::Output {
        let mut z = [0.; N];
        for ((zref, aval), bval) in z.iter_mut().zip(&self.0).zip(&rhs.0) {
            *zref = aval + bval;
        }
        Coord::<{ N }>(z)
    }
}

impl<const N: usize> core::ops::Add<Coord<{ N }>> for Coord<{ N }> {
    type Output = Coord<{ N }>;
    #[inline]
    fn add(self, rhs: Coord<{ N }>) -> Self::Output {
        let mut z = [0.; N];
        for ((zref, aval), bval) in z.iter_mut().zip(&self.0).zip(&rhs.0) {
            *zref = aval + bval;
        }
        Coord::<{ N }>(z)
    }
}

#[cfg(dormand_prince_logging)]
macro_rules! log {
    ($($arg:tt)*) => (eprint!($($arg)*));
}
#[cfg(dormand_prince_logging)]
macro_rules! logln {
    () => (log!("\n"));
    ($($arg:tt)*) => ({
        (eprintln!($($arg)*));
    })
}

#[allow(unused_macros)]
#[cfg(not(dormand_prince_logging))]
macro_rules! log {
    ($($arg:tt)*) => {};
}
#[cfg(not(dormand_prince_logging))]
macro_rules! logln {
    () => {
        log!("\n")
    };
    ($($arg:tt)*) => {};
}

fn set_initial_step<Eq: DifferentialEquation<{ N }>, const N: usize>(
    problem: &Eq,
    abs_tol: f64,
    rel_tol: f64,
    t0: f64,
    y0: &Coord<{ N }>,
) -> f64 {
    let order = 5;

    let f0 = problem.ode_dot_y(t0, y0).0;
    let scale: Vec<f64> = y0.0.iter().map(|y0| abs_tol + y0.abs() * rel_tol).collect();
    let d0 =
        y0.0.iter()
            .zip(scale.iter())
            .map(|(y0, scale)| (y0 / scale).powi(2) / N as f64)
            .sum::<f64>()
            .sqrt(); // in python, this is much cleaner: d0 = norm(y0 / scale)
    let d1 =
        f0.0.iter()
            .zip(scale.iter())
            .map(|(f0, scale)| (f0 / scale).powi(2) / N as f64)
            .sum::<f64>()
            .sqrt();
    let h0 = if d0 < 1e-5 || d1 < 1e-5 {
        1e-6
    } else {
        0.01 * d0 / d1
    };

    let direction = 1.0;
    let y1 = y0 + f0 * h0 * direction;
    let f1 = problem.ode_dot_y(t0 + h0 * direction, &y1).0;
    let d2 =
        f0.0.iter()
            .zip(f1.0.iter())
            .zip(scale.iter())
            .map(|((f0, f1), scale)| ((f1 - f0) / scale).powi(2) / N as f64)
            .sum::<f64>()
            .sqrt()
            / h0;

    let h1 = if d1 <= 1e-15 && d2 <= 1e-15 {
        1e-6_f64.max(h0 * 1e-3)
    } else {
        (0.01 / d1.max(d2)).powf(1. / (order as f64 + 1.))
    };

    h1.min(100. * h0)
}

#[inline]
fn dp_step<Eq: DifferentialEquation<{ N }>, const N: usize>(
    problem: &Eq,
    abs_tol: f64,
    rel_tol: f64,
    t: f64,
    y: &Coord<{ N }>,
    h: f64,
    cached: &Coord<{ N }>,
) -> (Coord<{ N }>, f64, Coord<{ N }>, bool) {
    let c = [0., 1. / 5., 3. / 10., 4. / 5., 8. / 9., 1.];
    let a = [
        [0., 0., 0., 0., 0.],
        [1. / 5., 0., 0., 0., 0.],
        [3. / 40., 9. / 40., 0., 0., 0.],
        [44. / 45., -56. / 15., 32. / 9., 0., 0.],
        [
            19372. / 6561.,
            -25360. / 2187.,
            64448. / 6561.,
            -212. / 729.,
            0.,
        ],
        [
            9017. / 3168.,
            -355. / 33.,
            46732. / 5247.,
            49. / 176.,
            -5103. / 18656.,
        ],
    ];
    let b = [
        35. / 384.,
        0.,
        500. / 1113.,
        125. / 192.,
        -2187. / 6784.,
        11. / 84.,
    ];
    let e = [
        -71. / 57600.,
        0.,
        71. / 16695.,
        -71. / 1920.,
        17253. / 339200.,
        -22. / 525.,
        1. / 40.,
    ];
    // e = b_i^* - b_i

    // Alternative implementation of the for loop below (scaled by h):
    // let k1 = problem.ode_eq_zero(t + c[0] * h, y) * h;
    // let k2 = problem.ode_eq_zero(t + c[1] * h, &(y + &k1 * a[1][0])) * h;
    // let k3 = problem.ode_eq_zero(t + c[2] * h, &(y + &k1 * a[2][0] + &k2 * a[2][1])) * h;
    // let k4 = problem.ode_eq_zero(
    //     t + c[3] * h,
    //     &(y + &k1 * a[3][0] + &k2 * a[3][1] + &k3 * a[3][2]),
    // ) * h;
    // let k5 = problem.ode_eq_zero(
    //     t + c[4] * h,
    //     &(y + &k1 * a[4][0] + &k2 * a[4][1] + &k3 * a[4][2] + &k4 * a[4][3]),
    // ) * h;
    // let k6 = problem.ode_eq_zero(
    //     t + c[5] * h,
    //     &(y + &k1 * a[5][0] + &k2 * a[5][1] + &k3 * a[5][2] + &k4 * a[5][3] + &k5 * a[5][4]),
    // ) * h;
    let mut ks = Vec::with_capacity(7);
    ks.push(*cached);
    for i in 1..6 {
        let this_y: Coord<{ N }> = ks
            .iter()
            .zip(a[i].iter().take(i))
            .map(|(k, a)| k * h * (*a as f64))
            .fold(*y, |acc, x| acc + x);
        ks.push(problem.ode_dot_y(t + c[i] * h, &this_y).0);
    }

    // Alternative implementation (scaled by h)
    //let y_new = y + &k1 * b[0] + &k2 * b[1] + &k3 * b[2] + &k4 * b[3] + &k5 * b[4] + &k6 * b[5];
    let y_new: Coord<{ N }> = ks
        .iter()
        .zip(b.iter())
        .map(|(k, a)| k * h * (*a as f64))
        .fold(*y, |acc, x| acc + x);

    let (k7, cont) = problem.ode_dot_y(t + h, &y_new);
    ks.push(k7);

    logln!("ks: {:?}", ks);

    // Alternative implementation (scaled by h)
    // let delta = &k1 * e[0] + &k2 * e[1] + &k3 * e[2] + &k4 * e[3] + &k5 * e[4] + &k6 * e[5] + &k7 * e[6];
    let delta = ks
        .iter()
        .zip(e.iter())
        .map(|(k, a)| k * h * (*a as f64))
        .fold(Coord::<N>([0.; N]), |acc, x| acc + x);

    let temp: f64 = delta
        .0
        .iter()
        .zip(&y.0)
        .zip(&y_new.0)
        .map(|((delta, y), y_new)| (delta / (abs_tol + y.abs().max(y_new.abs()) * rel_tol)).powi(2))
        .sum::<f64>();
    let err = (temp / N as f64).sqrt();

    (y_new, err, k7, cont)
}

/// Returned by [solve_ivp]
#[derive(Clone, Copy, Debug)]
pub enum IvpResult<const N: usize> {
    /// The final time `t_1` was successfully reached. The first field is
    /// `y(t_1)`.
    FinalTimeReached(Coord<{ N }>),
    /// The next timestep would be smaller than 10 machine epsilons, so
    /// integration stopped. The first field is the time `t_s` when integration
    /// stopped, the second field is `y(t_s)`.
    StepTooSmall(f64, Coord<{ N }>),
    /// If `problem.ode_dot_y(t,y).1 == false` for a pair of t,y that are part
    /// of the solution curve, this variant is returned. The first two fields
    /// are the last point where `problem.ode_dot_y(t,y).1 == true` and the last
    /// two fields are the next point where `problem.ode_dot_y(t,y).1 == false`.
    OdeRequestedExit(f64, Coord<{ N }>, f64, Coord<{ N }>),
    /// After every integration step, `callback(t,y)` is called. If it returns
    /// `false`, this variant is returned. The first two fields are the last
    /// point where `callback` returned `true` and the last two fields are the
    /// next point where `callback` returned `false`.
    CallbackRequestedExit(f64, Coord<{ N }>, f64, Coord<{ N }>),
}

/// Solves an initial value problem:
///
/// If you know `y(t_0)` and and you know a function `f(t, y)` so that ` dy/dt =
/// f(t, y) ∀ t ∈ ℝ`, this function can calculate `y(t_1)`.
/// # Arguments
///  * `problem`: The differential equation is `dy/dt = problem.ode_dot_y(t,
///    y).0`
///  * `tspan`: initial time `t_0` and final time `t_1`. `t_1>=t_0` is asserted.
///  * `y_0`: Inital state: `y_0=y(t_0)`
///  * `callback`: After each time step, `callback(t, y)` is called. If it
///    returns false, this solver will exit. Pass `|_,_| true` as `callback`
///    argument if you don't need this.
///  * `abs_tol`: Absolute Tolerance
///  * `rel_tol`: Relative Tolerance
///    # Return Value
/// See [IvpResult]
pub fn solve_ivp<Eq: DifferentialEquation<{ N }>, F, const N: usize>(
    problem: &Eq,
    tspan: (f64, f64),
    y_0: Coord<{ N }>,
    mut callback: F,
    abs_tol: f64,
    rel_tol: f64,
) -> IvpResult<N>
where
    F: FnMut(f64, &Coord<{ N }>) -> bool,
{
    // Potential performance improvements
    //  - call func directly instead of using a function pointer
    //  - while !step_accepted is probably really bad for the branch predictor
    //  - The more explicit, "alternative implementations" are faster
    assert!(tspan.1 >= tspan.0);

    let safety = 0.9;
    let fac_min = 0.2;
    let fac_max = 10.0;

    let mut h = set_initial_step(problem, abs_tol, rel_tol, tspan.0, &y_0);

    let mut t = tspan.0;
    let mut y = y_0;

    let mut cached = problem.ode_dot_y(t, &y).0;
    let mut cont = true;
    loop {
        if t >= tspan.1 {
            return IvpResult::FinalTimeReached(y);
        }

        let mut step_accepted = false;
        let mut step_rejected = false;

        let mut y_new = None;
        let mut t_new = None;

        let min_step = (t.next_after(std::f64::INFINITY) - t) * 10.;
        logln!("\n-------------------------------------");
        logln!("stepping t: {}\ty: {:?}", t, y.0);

        while !step_accepted {
            logln!("\ntrying h_abs = {}", h);
            if h < min_step {
                return IvpResult::StepTooSmall(t, y);
            }
            h = h.min(tspan.1 - t);
            t_new = Some(t + h);
            let temp = dp_step(problem, abs_tol, rel_tol, t, &y, h, &cached);
            y_new = Some(temp.0);
            cont = temp.3;
            let err: f64 = temp.1;
            logln!("err {}", err);
            if err < 1. {
                let mut factor = if err == 0. {
                    fac_max
                } else {
                    (safety * err.powf(-1. / 5.)).min(fac_max)
                };
                if step_rejected {
                    factor = factor.min(1.);
                }
                h *= factor;
                step_accepted = true;
                cached = temp.2;
            } else {
                h *= (safety * err.powf(-1. / 5.)).max(fac_min);
                step_rejected = true;
            }
        }
        let t_new = t_new.unwrap();
        let y_new = y_new.unwrap();
        if !cont {
            return IvpResult::OdeRequestedExit(t, y, t_new, y_new);
        }
        if !callback(t_new, &y_new) {
            return IvpResult::CallbackRequestedExit(t, y, t_new, y_new);
        }
        y = y_new;
        t = t_new;
    }
}
