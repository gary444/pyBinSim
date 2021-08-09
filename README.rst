
PyBinSim Server
========

This is an adapted version of pyBinSim which runs as a local audio processing server. The client sends audio packets via a TCP connection managed by the ZeroMQ library. PyBinSim applies binaural simulation processing, according to the spatial information delivered in the packet. The audio is then sent back, completing a request-reply communication model.


Install
-------
Miniconda should be used to manage dependencies. The installers for Miniconda can be found `here <https://docs.conda.io/en/latest/miniconda.html>`_.
::

    $ conda create --name binsim python=3.5 numpy pyzmq
    $ source activate binsim # or conda activate binsim on windows
    $ pip install cython
    $ pip install cffi==1.14.5
    $ pip install pybinsim
    
On linux, make sure that gcc and the development headers for libfftw and portaudio are installed, before invoking `pip install pybinsim`.
For ubuntu::

    $ apt-get install gcc portaudio19-dev libfftw3-dev
    

Run
---

Navigate to the root of the repository and run file run.py:

    $ python run.py

Configuration can be specified in the file ``config/server_config.cfg``.

Description
-------

For a description of pyBinSim please see the original `pyBinSim repository <https://github.com/pyBinSim/pyBinSim>`_.


Reference:
----------

Please cite the authors' work:

Neidhardt, A.; Klein, F.; Knoop, N. and Köllmer, T., "Flexible Python tool for dynamic binaural synthesis applications", 142nd AES Convention, Berlin, 2017.



