

## Installation

1. Install `zmqpp` according the to instructions from the page below. Skip `libsodium` part: do not clone it and do not use `--with-libsodium` flag. Skip `make check` and `make installcheck` steps

    https://github.com/zeromq/zmqpp#installation
2. Install `Python 3.9`

3. Clone this project and install dependencies from `requirements.txt`:

    `pip install -r requirements.txt`

4. Download and install `Matlab R2021b`

5. Install Matlab Engine for Python

    https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html

6. Download `MLScripts` and update its path in `bridge/common/config.py` if needed

7. Download `LARCmaCS` and checkout to `python` branch. Build it as usually

## Running

1. Run `simulator` and `game controller`

2. Run python version of `LARCmaCS`

2. Run bridge: `python main.py`
 # strategy
# strategy
