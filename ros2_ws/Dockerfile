# Start with the ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Add build arguments for user creation
ARG USER_NAME=rosdev
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Install all necessary dependencies as root in a single layer
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core tools
    git \
    sudo \
    openssh-client \
    curl \
    unzip \
    ca-certificates \
    wget \
    coreutils \
    python3-pip \
    # --- Recommended Additions ---
    gdb \
    iputils-ping \
    net-tools \
    usbutils \
    build-essential \
    nano \
    yq \
    # Qt/X11 runtime dependencies commonly required by Gazebo/Qt apps (xcb plugin deps)
    libxcb1 \
    libx11-xcb1 \
    libxcb-glx0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    # Download and install cloudflared
    && CLOUDFLARED_ARCH=$(dpkg --print-architecture | sed 's/amd64/amd64/g; s/arm64/arm64/g') \
    && wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-${CLOUDFLARED_ARCH}.deb -O cloudflared.deb \
    && apt-get install -y ./cloudflared.deb \
    && rm cloudflared.deb \
    # --- Cleanup ---
    && rm -rf /var/lib/apt/lists/*