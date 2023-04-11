FROM rust

WORKDIR /usr/src/myapp
COPY . .

RUN rustup target add thumbv7em-none-eabihf

RUN cargo install cargo-binutils

RUN rustup component add llvm-tools-preview

RUN apt-get update && apt-get install -y libudev-dev

RUN cargo install probe-run
