.PHONY: test build install clean bump-major bump-minor bump-patch

PYTHON = python3
PIP = pip3

test:
	${PYTHON} -m pytest -v

build: test
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
