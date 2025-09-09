FROM rust:1.88
ARG USER_ID=1000

RUN echo "user:x:$USER_ID:$USER_ID::/home/user:bash" >> /etc/passwd && echo "user:x:$USER_ID:" >> /etc/group && \
    mkdir -p /home/user && chown user:user /home/user && \
    echo "dialout:x:20:user" >> /etc/group && \
    echo "plugdev:x:46:user" >> /etc/group

RUN chown -R user:user /home/user

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
        libudev-dev libusb-1.0-0-dev && \
    DEBIAN_FRONTEND=noninteractive apt-get clean

USER user

WORKDIR /home/user/silpa-fw

RUN rustup target add thumbv7em-none-eabihf
RUN cargo install cargo-binutils && rustup component add llvm-tools-preview
RUN cargo install probe-rs-tools --locked

ENTRYPOINT ["bash"]
