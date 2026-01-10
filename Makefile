.PHONY: test build install clean

PYTHON = python3
PIP = pip3

test:
	$(PYTHON) tests/test_transform.py

build:
	$(PYTHON) -m build

install:
	$(PIP) install .

clean:
	rm -rf dist/ build/ *.egg-info
	find . -type d -name "__pycache__" -exec rm -rf {} +
