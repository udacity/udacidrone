.PHONY: lint format test FORCE

all: lint test

lint: FORCE
	flake8 examples/ udacidrone/ tests/

format: FORCE
	yapf -i --recursive *.py examples/ udacidrone/ tests/
	isort --recursive *.py examples/ udacidrone/ tests/

test: FORCE
	pytest -vx

FORCE: