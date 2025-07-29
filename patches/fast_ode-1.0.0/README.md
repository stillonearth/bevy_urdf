This crate is (afaik) the fastest implementation of the explicit Runge-Kutta
method of order 5(4) with the Dormand-Prince pair of formulas. It is
identical to [scipy's
implementation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.RK45.html).
This crate can be used to solve ordinary differential equations.

I do not use this crate anymore and I do not intend to increase the project scope, but I will address/fix every issue reported.
IMHO this is a small but very well written and documented crate.
## Example:
```rust
struct HarmonicOde {}
impl fast_ode::DifferentialEquation<2> for HarmonicOde {
    fn ode_dot_y(&self, _t: f64, y: &fast_ode::Coord<2>) -> (fast_ode::Coord<2>, bool) {
        let x = y.0[0];
        let v = y.0[1];
        (fast_ode::Coord::<2>([v, -x]), true)
    }
}
let ode = HarmonicOde {};
let res = fast_ode::solve_ivp(&ode, (0., 10.), fast_ode::Coord([0., 1.]), |_, _| true, 1e-6, 1e-3);
let numerical_sol = match res {
    fast_ode::IvpResult::FinalTimeReached(y) => y.0,
    _ => panic!(),
};
let theoretical_sol = [10_f64.sin(), 10_f64.cos()];
assert!(numerical_sol[0]-theoretical_sol[0] < 1e-2);
assert!(numerical_sol[1]-theoretical_sol[1] < 1e-2);
```
