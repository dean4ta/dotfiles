.DEFAULT_GOAL := help
include .env # include environment variables

XAUTH := /tmp/.docker.xauth
DISPLAY_FORWARDING_FLAGS := --env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=$(XAUTH)" \
	--volume="$(XAUTH):$(XAUTH)" \
	--volume="/tmp:/tmp:rw"
GPU_FLAGS := --gpus=all

DOCKER_RUN = docker run --rm -it\
	--privileged \
	--workdir $(SOURCE_MOUNT) \
	-v $(shell pwd)/bags:/root/bags \
	$(if $(FORWARD_X),$(DISPLAY_FORWARDING_FLAGS)) \
	$(if $(USE_GPU),$(GPU_FLAGS)) \
	$(CONTAINER)

.PHONY: help
help: ## Display this help message
	@grep -hE '^\S+:.*##' $(MAKEFILE_LIST) | sed -e 's/:[[:blank:]]*\(##\)[[:blank:]]*/\1/' | column -s '##' -t

.PHONY: up
up: ## Launch the container
	@$(DOCKER_RUN)

.PHONY: up-display
up-display: FORWARD_X := yes
up-display: ## Laucnh the container and forward the X display
	@xhost +local:root
	@xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $(XAUTH) nmerge -
	@$(DOCKER_RUN)
	@xhost -local:root

.PHONY: build
build: ## Build the container image
	@echo "Starting Docker build with the following command"
	@DOCKER_BUILDKIT=1 docker build \
		--build-arg GIT_USERNAME=$(GIT_USERNAME) \
		-t \
		$(CONTAINER) .
