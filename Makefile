REMOTE=omdev.local
ARTIFACT=build/firmware.elf
REMOTE_ARTIFACT=/tmp/firmware.elf
MICRO_ROS_AGENT_IMAGE=omros2firmware:microros
MICRO_ROS_DEVICE=/dev/ttyAMA0

PHONY: upload
upload:
	@echo "Uploading to $(REMOTE)"
	@scp "$(ARTIFACT)" $(REMOTE):"$(REMOTE_ARTIFACT)"
	@ssh $(REMOTE) bash -s flash "$(REMOTE_ARTIFACT)" < ./utils/remote-openocd.sh

PHONY: debug
debug:
	@ssh $(REMOTE) bash -s debug < ./utils/remote-openocd.sh

agent:
	@docker run --rm -it \
		-v /dev:/dev \
		-e "SERIAL_DEVICE=$(MICRO_ROS_DEVICE)"
		--network host \
		--name micro-ros-agent \
		$(MICRO_ROS_AGENT_IMAGE)

agent_remote:
	@./utils/remote-forward-agent-tcp.sh \
		$(REMOTE) \
		$(MICRO_ROS_DEVICE) \
		38123 \
		docker run --rm -it \
		--name micro-ros-agent \
		$(MICRO_ROS_AGENT_IMAGE) \
		tcp4 $(REMOTE) --port 38123