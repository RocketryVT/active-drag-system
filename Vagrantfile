#-----------------------------------------------------------------------
$bootstrap = <<BOOTSTRAP
export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get upgrade

# install base development tools
apt-get -y install build-essential
apt-get -y install cmake valgrind gcc-arm-none-eabi

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
