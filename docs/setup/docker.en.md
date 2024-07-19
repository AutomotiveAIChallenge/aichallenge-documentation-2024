# Installing the Virtual Environment

## Installing Dependencies

First, install the necessary libraries.

```bash
sudo apt update
sudo apt install -y python3-pip ca-certificates curl gnupg
```

## Installing Docker

Install Docker using the commands from the [official documentation](https://docs.docker.com/engine/install/ubuntu/).

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
newgrp docker
```

Verify that Docker is installed correctly with the following command:

```bash
sudo docker run hello-world
```

If you see `Hello from Docker!`, Docker is installed correctly.

## Installing Rocker

Rocker is a tool that simplifies running GUI applications inside Docker containers.

Although the [official README](https://github.com/osrf/rocker?tab=readme-ov-file#debians-recommended) recommends installing via apt, we will use pip for simplicity.

```bash
pip install rocker
```

By default, the path to the rocker executable is not included in the PATH, so add it to `.bashrc` with the following commands:

```bash
echo export PATH='$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

## Obtaining the Docker Image for the Autoware Environment

Download the Docker image for the Autoware environment used in the AI Challenge.

The Docker image is approximately 10GB in size, so it is recommended to use a wired LAN for downloading.

```bash
docker pull ghcr.io/automotiveaichallenge/autoware-universe:humble-latest
```

Check if the download was successful with the following command:

```bash
docker images
```

If the Docker image was downloaded correctly, you will see output similar to the following:

```txt
REPOSITORY                                        TAG                       IMAGE ID       CREATED         SIZE
ghcr.io/automotiveaichallenge/autoware-universe   humble-latest             30c59f3fb415   13 days ago     8.84GB
```

## Next Step

Two types of AWSIM are provided.

For first-time users, proceed to the documentation for headless AWSIM. If you have a PC with a GPU and want a richer development environment, proceed to the documentation for AWSIM with visualization.

[Download headless AWSIM](./headless-simulation.en.md)

[Download AWSIM with visualization](./visible-simulation.en.md)
