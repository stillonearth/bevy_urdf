use crate::*;
use assert_approx_eq::assert_approx_eq;
use core::f64::NAN;
use mathru::{
    algebra::linear::{Vector, General},
    analysis::differential_equation::ordinary::solver::explicit::runge_kutta::adaptive::{
        DormandPrince54, ProportionalControl,
    },
    analysis::differential_equation::ordinary::{ExplicitInitialValueProblemBuilder, ExplicitODE},
    vector,
};
use test::Bencher;
struct HarmonicOde {}
impl DifferentialEquation<2> for HarmonicOde {
    fn ode_dot_y(&self, _t: f64, y: &Coord<2>) -> (Coord<2>, bool) {
        let x = y.0[0];
        let v = y.0[1];
        (Coord::<2>([v, -x]), true)
    }
}
impl ExplicitODE<f64> for HarmonicOde {
    fn ode(&self, _t: &f64, y: &Vector<f64>) -> Vector<f64> {
        let a: General<f64> = General::new(2, 2, vec![0.0, 1.0, -1.0, 0.0]);
        &a * y
    }
}
struct CoupledHarmonicState {
    x1: f64,
    x2: f64,
    v1: f64,
    v2: f64,
}
impl From<[f64; 4]> for CoupledHarmonicState {
    fn from(ar: [f64; 4]) -> Self {
        CoupledHarmonicState {
            x1: ar[0],
            x2: ar[1],
            v1: ar[2],
            v2: ar[3],
        }
    }
}
impl From<CoupledHarmonicState> for [f64; 4] {
    fn from(s: CoupledHarmonicState) -> Self {
        [s.x1, s.x2, s.v1, s.v2]
    }
}

struct CoupledHarmonicStructOde {}
impl DifferentialEquation<4> for CoupledHarmonicStructOde {
    fn ode_dot_y(&self, _t: f64, y: &Coord<4>) -> (Coord<4>, bool) {
        let s = CoupledHarmonicState::from(y.0);
        (
            Coord::<4>([s.v1, s.v2, -s.x1 + 0.1 * s.x2, -s.x2 + 0.1 * s.x1]),
            true,
        )
    }
}

struct CoupledHarmonicArOde {}
impl DifferentialEquation<4> for CoupledHarmonicArOde {
    fn ode_dot_y(&self, _t: f64, y: &Coord<4>) -> (Coord<4>, bool) {
        let x1 = y.0[0];
        let x2 = y.0[1];
        let v1 = y.0[2];
        let v2 = y.0[3];
        (Coord::<4>([v1, v2, -x1 + 0.1 * x2, -x2 + 0.1 * x1]), true)
    }
}

struct YoungLaplaceOde {
    phi_0: f64,
    startsign: f64,
}
impl DifferentialEquation<2> for YoungLaplaceOde {
    fn ode_dot_y(&self, pmphi: f64, y: &Coord<2>) -> (Coord<2>, bool) {
        let phi = -pmphi * self.phi_0.signum();
        let x = y.0[0];
        let z = y.0[1];
        #[allow(clippy::float_cmp)]
        {
            if (x * z - phi.sin()).signum() != self.startsign {
                dbg!();
                return (Coord::<2>([NAN, NAN]), false);
            }
        }
        let dxdphi = x * phi.cos() / (x * z - phi.sin()); //Das ist unsere version der Gleichung (20) aus Shangs paper
        let dzdphi = x * phi.sin() / (x * z - phi.sin()); //Das ist unsere version der Gleichung (21) aus Shangs paper
        (
            Coord::<2>([-self.phi_0.signum() * dxdphi, -self.phi_0.signum() * dzdphi]),
            true,
        )
    }
}

struct YoungLaplace {
    phi_0: f64,
    x_contact: f64,
    z_contact: f64,
    startsign: f64,
}
impl ExplicitODE<f64> for YoungLaplace {
    fn ode(&self, pmphi: &f64, q: &Vector<f64>) -> Vector<f64> {
        let phi = -pmphi * self.phi_0.signum();
        let q = q.clone().convert_to_vec();
        let x = q[0];
        let z = q[1];
        #[allow(clippy::float_cmp)]
        {
            if (x * z - phi.sin()).signum() != self.startsign {
                return vector![NAN; NAN];
            }
        }
        let dxdphi = x * phi.cos() / (x * z - phi.sin()); //Das ist unsere version der Gleichung (20) aus Shangs paper
        let dzdphi = x * phi.sin() / (x * z - phi.sin()); //Das ist unsere version der Gleichung (21) aus Shangs paper
        vector![-self.phi_0.signum() * dxdphi; -self.phi_0.signum() * dzdphi]
    }
}
impl YoungLaplace {
    fn time_span(&self) -> (f64, f64) {
        (-self.phi_0.abs(), 0.)
    }
    fn init_cond(&self) -> Vector<f64> {
        #[allow(clippy::float_cmp)]
        // It's ok, because startsign is the output of the signum function
        {
            assert!(self.startsign == 1. || self.startsign == -1.); // Is this the correct place for this assertion?
        }
        vector![self.x_contact; self.z_contact]
    }
}

#[test]
fn compare_with_mathru() {
    let phi_0 = -0.7653981633974483;
    let x_contact = 0.34641208585537364;
    let z_contact = 0.4145142430976436;

    let h_0 = 0.09026305092159702;
    let fac = 0.9;
    let fac_min = 0.2; // 0.01;
    let fac_max = 10.0; // 2.0;
    let n_max = 1e9 as u32;
    let abs_tol = 1e-6;
    let rel_tol = 1e-3;

    let solver = ProportionalControl::new(n_max, h_0, fac, fac_min, fac_max, abs_tol, rel_tol);

    let ode = YoungLaplaceOde {
        phi_0,
        startsign: (x_contact * z_contact - phi_0.sin()).signum(),
    };
    let myres = solve_ivp(
        &ode,
        (-phi_0.abs(), 0.),
        Coord([x_contact, z_contact]),
        |_, _| true,
        1e-6,
        1e-3,
    );

    let ode = YoungLaplace {
        phi_0,
        x_contact,
        z_contact,
        startsign: (x_contact * z_contact - phi_0.sin()).signum(),
    };
    let problem = ExplicitInitialValueProblemBuilder::new(&ode, ode.time_span().0, ode.init_cond())
        .t_end(ode.time_span().1)
        .build();
    let (_pm_phi, xz) = solver.solve(&problem, &DormandPrince54::default()).unwrap();

    let myres_y = match myres {
        IvpResult::FinalTimeReached(y) => y,
        _ => panic!(),
    };
    let res = xz[xz.len() - 1].clone().convert_to_vec();
    assert_approx_eq!(myres_y.0[0], res[0], 1e-3);
    assert_approx_eq!(myres_y.0[1], res[1], 1e-3);
}

#[bench]
fn b_mathru_young_laplace(b: &mut Bencher) {
    b.iter(|| {
        let phi_0 = -0.7653981633974483;
        let x_contact = 0.34641208585537364;
        let z_contact = 0.4145142430976436;

        let h_0 = 0.09026305092159702;
        let fac = 0.9;
        let fac_min = 0.2; // 0.01;
        let fac_max = 10.0; // 2.0;
        let n_max = 1e9 as u32;
        let abs_tol = 1e-6;
        let rel_tol = 1e-3;

        let solver = ProportionalControl::new(n_max, h_0, fac, fac_min, fac_max, abs_tol, rel_tol);
        let ode = YoungLaplace {
            phi_0,
            x_contact,
            z_contact,
            startsign: (x_contact * z_contact - phi_0.sin()).signum(),
        };
        let problem = ExplicitInitialValueProblemBuilder::new(&ode, ode.time_span().0, ode.init_cond())
        .t_end(ode.time_span().1)
        .build();
        solver.solve(&problem, &DormandPrince54::default()).unwrap()
    });
}

#[bench]
fn b_young_laplace(b: &mut Bencher) {
    b.iter(|| {
        let phi_0 = -0.7653981633974483;
        let x_contact = 0.34641208585537364;
        let z_contact = 0.4145142430976436;
        let ode = YoungLaplaceOde {
            phi_0,
            startsign: (x_contact * z_contact - phi_0.sin()).signum(),
        };
        solve_ivp(
            &ode,
            (-phi_0.abs(), 0.),
            Coord([x_contact, z_contact]),
            |_, _| true,
            1e-6,
            1e-3,
        )
    });
}

#[bench]
fn b_mathru_harmonic(b: &mut Bencher) {
    b.iter(|| {
        let h_0 = 0.09026305092159702;
        let fac = 0.9;
        let fac_min = 0.2; // 0.01;
        let fac_max = 10.0; // 2.0;
        let n_max = 1e9 as u32;
        let abs_tol = 1e-6;
        let rel_tol = 1e-3;

        let solver = ProportionalControl::new(n_max, h_0, fac, fac_min, fac_max, abs_tol, rel_tol);
        let ode = HarmonicOde {};
        let problem = ExplicitInitialValueProblemBuilder::new(&ode, 0.0, vector![1.; 0.])
            .t_end(10.0)
            .build();
        solver.solve(&problem, &DormandPrince54::default()).unwrap()
    });
}

#[bench]
fn b_harmonic(b: &mut Bencher) {
    b.iter(|| {
        let ode = HarmonicOde {};
        solve_ivp(&ode, (0., 10.), Coord([1., 0.]), |_, _| true, 1e-6, 1e-3)
    });
}

#[bench]
fn b_coupled_array(b: &mut Bencher) {
    b.iter(|| {
        let ode = CoupledHarmonicArOde {};
        solve_ivp(
            &ode,
            (0., 100.),
            Coord([1., 0., 0., 0.5]),
            |_, _| true,
            1e-6,
            1e-3,
        )
    });
}
#[bench]
fn b_coupled_struct(b: &mut Bencher) {
    b.iter(|| {
        let ode = CoupledHarmonicStructOde {};
        solve_ivp(
            &ode,
            (0., 100.),
            Coord([1., 0., 0., 0.5]),
            |_, _| true,
            1e-6,
            1e-3,
        )
    });
}
