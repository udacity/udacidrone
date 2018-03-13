---
id: dev-setup
title: Development Setup
sidebar_label: Setup
---

The easiest way to get setup for development is clone to clone `udacidrone` and add it to the `PYTHONPATH` environment variable.

Clone repo:

```
git clone https://github.com/udacity/udacidrone.git
```

Update `PYTHONPATH` environment variable:

```
export PYTHONPATH=path/to/udacidrone:$PYTHONPATH
```

## Working On The Code

The branch you're working on will be tested with CI, any lint or test errors will cause the build to fail.
We use `flake8` for linting, `yapf` for formatting, and `pytest` for testing. The configuration can be found in
[setup.cfg](https://github.com/udacity/udacidrone/blob/master/setup.cfg). The [Makefile](https://github.com/udacity/udacidrone/blob/master/Makefile) provides utilities to lint, format and test locally:

Linting:

```
make lint
```

Formatting:

```
make format
```

Testing:

```
make test
```

## Working On The Website

The website is built using [Docusaurus](https://docusaurus.io/). Everything website related can be found in the [`docs`](https://github.com/udacity/udacidrone/tree/master/docs) and [`website`](https://github.com/udacity/udacidrone/tree/master/website) directories.
