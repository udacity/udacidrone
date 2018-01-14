.PHONY: install lint format test clean FORCE

all: lint test

# install: FORCE
# 	pip install -e .

lint: FORCE
	flake8 examples/ udacidrone/ tests/

format: FORCE
	yapf -i --recursive *.py examples/ udacidrone/ tests/
	isort --recursive *.py examples/ udacidrone/ tests/

test: FORCE
	pytest -vx

FORCE: