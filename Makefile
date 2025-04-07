REMOTE=omdev.local
ARTIFACT=build/firmware.elf
REMOTE_ARTIFACT=/tmp/firmware.elf
MICRO_ROS_AGENT_IMAGE=ghcr.io/jkaflik/omros2-firmware:micro-ros
MICRO_ROS_DEVICE=/dev/ttyAMA0

PHONY: upload
upload:
	@echo "Uploading to $(REMOTE)"
	@scp "$(ARTIFACT)" $(REMOTE):"$(REMOTE_ARTIFACT)"
	@ssh $(REMOTE) bash -s flash "$(REMOTE_ARTIFACT)" < ./utils/remote-openocd.sh

PHONY: debug
debug:
	@ssh -t $(REMOTE) bash -s debug < ./utils/remote-openocd.sh

agent:
	@./utils/run-micro-ros-agent.sh 

agent_remote:
	@ssh -t $(REMOTE) bash -s $(MICRO_ROS_DEVICE) < ./utils/run-micro-ros-agent.sh 