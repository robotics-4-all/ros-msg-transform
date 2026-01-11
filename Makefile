.PHONY: test build install clean bump-major bump-minor bump-patch publish

PYTHON = python3
PIP = pip3

test:
	${PYTHON} -m pytest -v

test-docker:
	docker run --rm \
		-v $$(pwd):/workspace \
		-w /workspace \
		ros:noetic-ros-base \
		/bin/bash -c "apt-get update -qq && apt-get install -y -qq python3-pip && pip3 install .[test] && python3 -m pytest -v"

build:
	$(PYTHON) -m build

install: build
	$(PIP) install .

clean:
	rm -rf dist/ build/ *.egg-info
	find . -type d -name "__pycache__" -exec rm -rf {} +

bump-major:
	bumpversion major

bump-minor:
	bumpversion minor

bump-patch:
	bumpversion patch

publish: build
	uv publish
