
cargo test b_harmonic -- --nocapture
cargo bench tests::b_harmonic -- --exact

cargo rustc --release -- --emit=llvm-ir --emit=asm
cargo rustc --release -- -C target-cpu=native
export RUSTFLAGS='-C target-cpu=native'

export RUSTFLAGS="-C target-cpu=native -C profile-generate=/tmp/pgo-data"
cargo bench tests::b_coupled
llvm-profdata merge -o /tmp/pgo-data/merged.profdata /tmp/pgo-data

export RUSTFLAGS="-C target-cpu=native -C profile-use=/tmp/pgo-data/merged.profdata"
cargo bench tests::b_coupled