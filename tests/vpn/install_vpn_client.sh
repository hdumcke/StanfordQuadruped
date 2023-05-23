#!/bin/bash

sudo apt update
sudo apt install -y wireguard

wg genkey | sudo tee /etc/wireguard/privatekey | wg pubkey | sudo tee /etc/wireguard/publickey

# allow ipv4 forwarding
sudo sed -i "s/#net.ipv4.ip_forward/net.ipv4.ip_forward/" /etc/sysctl.conf

cat > /tmp/wg0.conf << EOF
[Interface]
PrivateKey = $(sudo cat /etc/wireguard/privatekey)
Address = 10.0.0.2/24

[Peer]
PublicKey = SERVERKEY
AllowedIPs = 10.0.0.0/24
Endpoint = SERVERIP:51820
EOF

sudo cp /tmp/wg0.conf /etc/wireguard/

sudo chmod 600 /etc/wireguard/{privatekey,wg0.conf}

cat > ~/readme-vpn << EOF
Connect to the server and do
sudo wg

This will show you the publik key of the server

On the client edit /etc/wireguard/wg0.conf
Replace SERVERKEY and SERVERIP

Get the client public key with this command on the client
sudo cat /etc/wireguard/publickey

On the server edit ~/add-peer.sh abd replace PEERPUBKEY with the publick key of the client. Then run:
~/add-peer.sh

Finally establish a VPN connection by running the following command on the client:

sudo wg-quick up wg0
EOF

echo "export CYCLONEDDS_URI=file:///home/ubuntu/multipass-orchestrator-configurations/wireguard-vpn/cyclone_dds_config_client.xml" >> ~/.bashrc
