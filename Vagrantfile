#-----------------------------------------------------------------------
$bootstrap = <<BOOTSTRAP
export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get upgrade

# install base development tools
apt-get -y install build-essential
apt-get -y install cmake valgrind
apt-get -y install crossbuild-essential-armhf
apt-get -y install libboost-all-dev
apt-get -y install libasio-dev
wget https://musl.cc/arm-linux-musleabihf-cross.tgz
tar -xzvf arm-linux-musleabihf-cross.tgz
echo 'export PATH="\$PATH:/home/vagrant/arm-linux-musleabihf-cross/bin"' >> .bashrc
rm -r arm-linux-musleabihf-cross.tgz

BOOTSTRAP
#-----------------------------------------------------------------------


# Configuration
Vagrant.configure("2") do |config|
  config.vm.box = "debian/bullseye64"
  config.vm.box_version = "11.20230615.1"

  # (default timeout is 300 s)
  config.vm.boot_timeout = 600

  # set up the VM
  config.vm.provision "shell", inline: $bootstrap
end
