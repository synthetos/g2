# Use a Debian-based image as a parent image
FROM debian:buster-slim

# Install build-essential, git, and uucp
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y build-essential git wget xz-utils uucp bossa-cli

# Set the working directory to /app
WORKDIR /app

# Clone the g2 GitHub repo
RUN git clone https://github.com/synthetos/g2.git && \
    cd g2 && \
    git submodule init && \
    git submodule update

RUN mkdir -p /app/g2/Motate/Tools/linux && \
    wget -P /app/g2/Motate/Tools/linux https://github.com/synthetos/gcc-arm-none-eabi-archive/releases/download/12.2.Rel1/arm-gnu-toolchain-12.2.rel1-linux-x86_64-arm-none-eabi.tar.xz



#Extract the toolchain archive to the directory

RUN cd /app/g2/Motate/Tools/linux && \
    tar xf arm-gnu-toolchain-12.2.rel1-linux-x86_64-arm-none-eabi.tar.xz && \
    rm -f "arm-gnu-toolchain-12.2.rel1-linux-x86_64-arm-none-eabi.tar.xz"

RUN apt-get install dos2unix

# Set the TERM environment variable
ENV TERM=xterm

# Set the working directory to the cloned repo directory
WORKDIR /app/g2/g2core

# Create a volume for storing build output
VOLUME /app/g2/g2core/bin 

COPY entrypoint.sh /app/entrypoint.sh
RUN dos2unix /app/entrypoint.sh
ENTRYPOINT ["/app/entrypoint.sh"]


# Define the default command to run the build
#CMD ["BOARD=gShield", "SETTINGS=default_settings.h"]