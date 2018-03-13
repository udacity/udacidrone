# Contributing to Udacidrone

If your contribution is:

1. **New Feature**. Open a pull request, this should initially be a proposal describing the feature. Once we agree everything looks good you may go ahead and implement it.
2. **Fix an FCND Issue**. Issues from the [FCND issues repo](https://github.com/udacity/fcnd-issue-reports/issues). The issues will be labelled `udacidrone`. Open a pull request with the issue fix and reference the issue in your PR.

If you are not familiar with creating a Pull Request, here are some guides:

- http://stackoverflow.com/questions/14680711/how-to-do-a-github-pull-request
- https://help.github.com/articles/creating-a-pull-request/

## Developing locally

1. Uninstall existing udacidrone

```
pip uninstall udacidrone
# if the above doesn't work
pip uninstall https://github.com/udacity/udacidrone.git
```

2. Clone the repo

```
git clone https://github.com/udacity/udacidrone.git
cd udacidrone
```

3. Setup develop environment

This mode will symlink the python files from the current local source tree into the
python install.

```
python setup.py build develop
```

Alternatively, you can add udacidrone to your `PYTHONPATH` environment variable.

```
export PYTHONPATH=path/to/udacidrone
```

Either of these modes allow editing files and updating udacidrone on the fly.


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
